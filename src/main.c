#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdint.h>

#include <mavlink/common/mavlink.h>

const char *path = "/dev/ttyAMA0";
const speed_t baud = B921600;

int main() {
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

    mavlink_status_t status;
    mavlink_message_t msg;
    int chan = MAVLINK_COMM_0;

    ssize_t n;
    uint8_t byte;
    while (1) {
        while ((n = read(fd, &byte, 1)) > 0) {
            if (mavlink_parse_char(chan, byte, &msg, &status)) {
                printf("received message\n");
            }
        }
    }

    close(fd);
    return 0;
}