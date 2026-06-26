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

static void *heartbeat_routine(void *arg) {
    struct timespec t1, t2;
    uint64_t elapsed_nsec;
    int fd = *(int *)arg;

    while (1) {
        clock_gettime(CLOCK_MONOTONIC, &t1);

        // stub

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

int main() {
    int fd = tty_init();
    if (fd < 0) return 1;

    pthread_t heartbeat_thread;
    int err_num = pthread_create(&heartbeat_thread, NULL, heartbeat_routine, &fd);
    if (err_num != 0) {
        fprintf(stderr, "pthread_create: could not create a thread (error %d)", err_num);
    }
    printf("heartbeat thread launched\n");

    printf("listening for incoming messages...\n\n");
    while (1) {
        mavlink_message_t msg;
        if (parse_stream(fd, &msg)) {
            uint8_t *msgptr = (uint8_t *)&msg;
            printf("len=%u seq=%u sid=%u cid=%u mid=%u\n",
                msg.len, msg.seq, msg.sysid, msg.compid, msg.msgid);
        }
    }

    close(fd);
    return 0;
}