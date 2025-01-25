#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/usbdevice_fs.h>

int main(int argc, char **argv) {
    const char *filename;
    int fd;

    if (argc < 2) {
        fprintf(stderr, "Usage: %s <device-file>\n", argv[0]);
        return 1;
    }
    filename = argv[1];

    fd = open(filename, O_WRONLY);
    if (fd < 0) {
        perror("Error opening device");
        return 1;
    }
    if (ioctl(fd, USBDEVFS_RESET, 0) < 0) {
        perror("Error in ioctl");
        close(fd);
        return 1;
    }
    close(fd);
    printf("Reset successful\n");
    return 0;
}
