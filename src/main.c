#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdint.h>
#include <time.h>
#include <pthread.h>
#include <stdlib.h>

#include <mavlink/common/mavlink.h>

#define MAVLINK_MAX_PKT_LEN 280

const char *path = "/dev/ttyAMA0";
const speed_t baud = B921600;
const uint64_t heartbeat_interval_nsec = 1000000000ULL; // 1000 ms

static void send_heartbeat(int fd) {
    mavlink_message_t msg;
    static uint8_t *buf[MAVLINK_MAX_PKT_LEN];

    uint8_t system_id = 255; // `GCS typically use a default system ID up around 255`
    uint8_t component_id = MAV_COMP_ID_ONBOARD_COMPUTER; // `By convention components have a default ID using the value as defined in MAV_COMPONENT`
    uint8_t type = MAV_TYPE_GCS; // from MAV_TYPE
    uint8_t autopilot = MAV_AUTOPILOT_INVALID; // `No valid autopilot, e.g. a GCS or other MAVLink component`
    uint8_t base_mode = 0; // couldn't find reliable source of what GCS should set
    uint8_t custom_mode = 0; // couldn't find reliable source of what GCS should set
    uint8_t system_status = 0; // couldn't find reliable source of what GCS should set

    mavlink_msg_heartbeat_pack(system_id, component_id, &msg, type, autopilot, base_mode, custom_mode, system_status);

    uint16_t msg_size = mavlink_msg_to_send_buffer((uint8_t *)buf, &msg);

    ssize_t bytes_written = write(fd, &buf, msg_size);

    if (bytes_written <= 0) {
        perror("write");
    }
    else if (bytes_written < msg_size) {
        fprintf(stderr, "warning: %zd bytes was written, but requested number is %u", bytes_written, msg_size);
    }
}

static void *heartbeat_routine(void *arg) {
    struct timespec t1, t2;
    uint64_t elapsed_nsec;
    int fd = *(int *)arg;

    while (1) {
        clock_gettime(CLOCK_MONOTONIC, &t1);

        send_heartbeat(fd);

        clock_gettime(CLOCK_MONOTONIC, &t2);
        
        elapsed_nsec = (uint64_t)(t2.tv_sec - t1.tv_sec) * 1000000000ULL +
            (uint64_t)(t2.tv_nsec - t1.tv_nsec);
        
        if (elapsed_nsec < heartbeat_interval_nsec) {
            uint64_t sleep_nsec = heartbeat_interval_nsec - elapsed_nsec;
            struct timespec ts = {
                .tv_sec = sleep_nsec / 1000000000ULL,
                .tv_nsec = sleep_nsec % 1000000000ULL,
            };
            nanosleep(&ts, NULL);
        }
    }

    return NULL;
}

int tty_init() {
    int fd = open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        perror("open");
        fprintf(stderr, "note: specified path was %s\n", path);
        close(fd);
        return -1;
    }
    if (!isatty(fd)) {
        perror("isatty");
        close(fd);
        return -1;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr: get tty parameters failed");
        close(fd);
        return -1;
    }

    tty.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON | IXOFF | IXANY);
    tty.c_lflag &= ~(ECHO | ECHOE | ECHONL | ICANON | IEXTEN | ISIG);
    tty.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
    tty.c_cflag |= CS8 | CREAD | CLOCAL;
    tty.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OLCUC | OPOST);
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 0;

    if (cfsetispeed(&tty, baud) != 0 ||
        cfsetospeed(&tty, baud) != 0) {
        perror("cfset[i/o]speed");
        close(fd);
        return -1;
    }

    if (tcflush(fd, TCIOFLUSH) != 0) perror("tcflush");
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        close(fd);
        return -1;
    }

    return fd;
}

void message_hexdump(const mavlink_message_t *msg)
{
    uint8_t buf[MAVLINK_MAX_PKT_LEN];
    uint16_t msg_size = mavlink_msg_to_send_buffer(buf, msg);

    printf("msgid=%u len=%u: ", msg->msgid, msg_size);
    for (uint16_t i = 0; i < msg_size; i++) {
        printf("%02X ", buf[i]);
    }
    printf("\n");
}

void message_decode_and_print(const mavlink_message_t *msg) {
    switch (msg->msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT: {
            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(msg, &hb);
            printf("HEARTBEAT: type=%u autopilot=%u base_mode=%u custom_mode=%u system_status=%u\n\n",
                   hb.type,
                   hb.autopilot,
                   hb.base_mode,
                   hb.custom_mode,
                   hb.system_status);
        } break;
        default: {
            message_hexdump(msg);
        } break;
    }
}

void message_receive(int fd) {
    mavlink_status_t status;
    mavlink_message_t msg;
    int chan = MAVLINK_COMM_0;

    ssize_t n;
    uint8_t byte;
    while ((n = read(fd, &byte, 1)) > 0) {
        if (mavlink_parse_char(chan, byte, &msg, &status)) {
            message_decode_and_print(&msg);
        }
    }
}

int main() {
    int fd = tty_init();
    if (fd < 0) {
        return 1;
    }

    pthread_t heartbeat_thread;
    int err_num = pthread_create(&heartbeat_thread, NULL, heartbeat_routine, (void *)&fd);
    if (err_num != 0) {
        fprintf(stderr, "pthread_create: could not create a thread (error %d)", err_num);
    }
    printf("heartbeat thread launched\n");

    printf("listening for incoming messages...\n\n");
    while (1) {
        message_receive(fd);
    }

    close(fd);
    return 0;
}