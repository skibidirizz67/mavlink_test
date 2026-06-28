#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdint.h>
#include <time.h>
#include <pthread.h>
#include <stdlib.h>
#include <stdbool.h>

#include <mavlink/common/mavlink.h>

#define MAVLINK_MAX_PKT_LEN 280

const uint64_t heartbeat_interval_nsec = 1000000000ULL; // 1000 ms
const char *path = "/dev/ttyAMA0";
const speed_t baud = B921600;

int tty_init() {
    int fd = open(path, O_RDWR | O_NOCTTY); // O_NONBLOCK
    if (fd < 0) {
        perror("[ERROR] open");
        fprintf(stderr, "[DEBUG] specified path was %s\n", path);
        close(fd);
        return -1;
    }
    if (!isatty(fd)) {
        perror("[ERROR] isatty");
        close(fd);
        return -1;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        perror("[ERROR] tcgetattr: get tty parameters failed");
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
        perror("[ERROR] cfset[i/o]speed");
        close(fd);
        return -1;
    }

    if (tcflush(fd, TCIOFLUSH) != 0) perror("[ERROR] tcflush");
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("[ERROR] tcsetattr");
        close(fd);
        return -1;
    }

    return fd;
}

static void *heartbeat_routine(void *arg) {
    struct timespec t1, t2;
    uint64_t elapsed_nsec;
    int fd = *(int *)arg;

    while (1) {
        clock_gettime(CLOCK_MONOTONIC, &t1);

        static mavlink_message_t msg;
        mavlink_msg_heartbeat_pack(
            255, MAV_COMP_ID_ONBOARD_COMPUTER, &msg,
            MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, MAV_STANDARD_MODE_NON_STANDARD, 0, MAV_STATE_ACTIVE
        );

        static uint8_t buf[MAVLINK_MAX_PKT_LEN];
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

        ssize_t written = write(fd, buf, len);
        if (written != (ssize_t)len) {
            perror("[WARNING] write");
            fprintf(stderr, "[DEBUG] %ld != %d\n", written, len);
        }

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

int parse_stream(int fd, mavlink_message_t *msg) {
    uint8_t buf[MAVLINK_MAX_PKT_LEN * 2];
    ssize_t n = read(fd, buf, MAVLINK_MAX_PKT_LEN * 2);
    mavlink_status_t status;

    for (ssize_t i = 0; i < n; i++) {
        if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], msg, &status)) {
            return 1;
        }
    }

    return 0;
}

void print_subscribed(mavlink_message_t *msg) {
    switch (msg->msgid) {
        case MAVLINK_MSG_ID_SYS_STATUS: {
            mavlink_sys_status_t pld;
            mavlink_msg_sys_status_decode(msg, &pld);
            printf("SYS_STATUS: onboard_control_sensors_present=%04X onboard_control_sensors_enabled=%04x onboard_control_sensors_health=%04X load=%u voltage_battery=%u current_battery=%d battery_remaining=%d drop_rate_comm=%u\n",
                pld.onboard_control_sensors_present, pld.onboard_control_sensors_enabled, pld.onboard_control_sensors_health, pld.load, pld.voltage_battery, pld.current_battery, pld.battery_remaining, pld.drop_rate_comm);
        } return;
        case MAVLINK_MSG_ID_ATTITUDE: {
            mavlink_attitude_t pld;
            mavlink_msg_attitude_decode(msg, &pld);
            printf("ATTITUDE: roll=%.3f pitch=%.3f yaw=%.3f rollspeed=%.3f pitchspeed=%.3f yawspeed=%.3f\n",
                pld.roll, pld.pitch, pld.yaw, pld.rollspeed, pld.pitchspeed, pld.yawspeed);
        } return;
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
            mavlink_global_position_int_t pld;
            mavlink_msg_global_position_int_decode(msg, &pld);
            printf("GLOBAL_POSITION_INT: time_boot_ms=%u lat=%d lon=%d alt=%d relative_alt=%d vx=%d vy=%d vz=%d hdg=%u\n",
                pld.time_boot_ms, pld.lat, pld.lon, pld.alt, pld.relative_alt, pld.vx, pld.vy, pld.vz, pld.hdg);
        } return;
        case MAVLINK_MSG_ID_AUTOPILOT_VERSION: {
            mavlink_autopilot_version_t pld;
            mavlink_msg_autopilot_version_decode(msg, &pld);
            printf("AUTOPILOT_VERSION: capabilities=%08lX flight_sw_version=%u middleware_sw_version=%u os_sw_version=%u board_version=%u vendor_id=%u product_id=%u uid=%lu",
                pld.capabilities, pld.flight_sw_version, pld.middleware_sw_version, pld.os_sw_version, pld.board_version, pld.vendor_id, pld.product_id, pld.uid);
        } return;
    }
} 

static void *listen_routine(void *arg) {
    uint8_t *buf = (uint8_t *)arg;
    uint8_t target_system = buf[0];
    uint8_t target_component = buf[1];
    int fd = ((uint32_t *)(buf+2))[0];

    while (1) {
        mavlink_message_t msg;
        if (!parse_stream(fd, &msg)) continue;
        if (msg.sysid != target_system || msg.compid != target_component) continue;

        printf("len=%u seq=%u sysid=%u compid=%u msgid=%u\n",
            msg.len, msg.seq, msg.sysid, msg.compid, msg.msgid);

        print_subscribed(&msg);
    }

    return NULL;
}

int main() {
    int fd = tty_init();
    if (fd < 0) return 1;

    pthread_t heartbeat_thread;
    int err_num = pthread_create(&heartbeat_thread, NULL, heartbeat_routine, &fd);
    if (err_num != 0) {
        fprintf(stderr, "[ERROR] pthread_create: could not create a thread (error %d)", err_num);
        return 1;
    }
    printf("[INFO] own heartbeat launched\n");

    printf("[INFO] waiting for FC heartbeat...\n");
    uint8_t target_system = 0;
    uint8_t target_component = 0;
    while (target_system == 0 || target_component == 0) {
        mavlink_message_t msg;
        mavlink_heartbeat_t hb;
        if (!parse_stream(fd, &msg)) continue;
        if (msg.msgid != MAVLINK_MSG_ID_HEARTBEAT) continue;
        mavlink_msg_heartbeat_decode(&msg, &hb);
        if (hb.autopilot == MAV_AUTOPILOT_INVALID) continue;
        // mavlink_version

        target_system = msg.sysid;
        target_component = msg.compid;
    }
    printf("[INFO] sysid=%u; compid=%u\n", target_system, target_component);

    pthread_t listen_thread;
    uint8_t buf[6] = {0};
    buf[0] = target_system;
    buf[1] = target_component;
    memcpy(buf + 2, (void *)&fd, 4); 

    err_num = pthread_create(&listen_thread, NULL, listen_routine, (void *)buf);
    if (err_num != 0) {
        fprintf(stderr, "[ERROR] pthread_create: could not create a thread (error %d)", err_num);
        return 1;
    }
    printf("[INFO] listening for incoming messages...\n\n");

    close(fd);
    return 0;
}