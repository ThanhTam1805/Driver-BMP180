#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <stdint.h>

// Định nghĩa các macro IOCTL giống trong kernel driver
#define IOCTL_MAGIC           'b'
#define IOCTL_READ_TEMP       _IOR(IOCTL_MAGIC, 0, long)
#define IOCTL_READ_PRESSURE   _IOWR(IOCTL_MAGIC, 1, struct bmp180_pressure_data)

// Cấu trúc dữ liệu cho áp suất (phải khớp với kernel)
struct bmp180_pressure_data {
    uint8_t oss;
    long pressure;
};

int main() {
    int fd;
    long temperature;
    struct bmp180_pressure_data pressure_data;
    
    // Mở device file
    fd = open("/dev/bmp180", O_RDWR);
    if (fd < 0) {
        perror("Failed to open the device");
        return EXIT_FAILURE;
    }

    // Đọc nhiệt độ
    if (ioctl(fd, IOCTL_READ_TEMP, &temperature) < 0) {
        perror("Failed to read temperature");
        close(fd);
        return EXIT_FAILURE;
    }
    printf("Temperature: %.1f °C\n", temperature / 10.0);

    // Đọc áp suất với oversampling setting = 3 (độ chính xác cao nhất)
    pressure_data.oss = 0;
    if (ioctl(fd, IOCTL_READ_PRESSURE, &pressure_data) < 0) {
        perror("Failed to read pressure");
        close(fd);
        return EXIT_FAILURE;
    }
    printf("Pressure: %.2f Pa\n", pressure_data.pressure/1.0);

    close(fd);
    return EXIT_SUCCESS;
}
