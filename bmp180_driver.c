
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/of.h>

#define DRIVER_NAME "bmp180_driver"
#define CLASS_NAME "bmp180_class"
#define DEVICE_NAME "bmp180"

#define BMP180_REG_CONTROL     0xF4
#define BMP180_REG_RESULT      0xF6
#define BMP180_REG_CAL_AC1     0xAA
#define BMP180_REG_CAL_AC2     0xAC
#define BMP180_REG_CAL_AC3     0xAE
#define BMP180_REG_CAL_AC4     0xB0
#define BMP180_REG_CAL_AC5     0xB2
#define BMP180_REG_CAL_AC6     0xB4
#define BMP180_REG_CAL_B1      0xB6
#define BMP180_REG_CAL_B2      0xB8
#define BMP180_REG_CAL_MB      0xBA
#define BMP180_REG_CAL_MC      0xBC
#define BMP180_REG_CAL_MD      0xBE
#define BMP180_REG_TEMPERATURE 0x2E
#define BMP180_REG_PRESSURE    0x34

#define IOCTL_MAGIC           'b'
#define IOCTL_READ_TEMP       _IOR(IOCTL_MAGIC, 0, long)
#define IOCTL_READ_PRESSURE   _IOWR(IOCTL_MAGIC, 1, struct bmp180_pressure_data)

struct bmp180_pressure_data {
    uint8_t oss;
    long pressure;
};

static struct i2c_client *bmp180_client = NULL;
static struct class *bmp180_class = NULL;
static struct device *bmp180_device= NULL;

static int major_number;
static DEFINE_MUTEX(bmp180_mutex);

static short AC1, AC2, AC3, B1, B2, MB, MC, MD;
static unsigned short AC4, AC5, AC6;
static long B5;

static s16 bmp180_read_s16(u8 reg)
{
    s16 msb = i2c_smbus_read_byte_data(bmp180_client, reg);
    s16 lsb = i2c_smbus_read_byte_data(bmp180_client, reg + 1);
    return (msb << 8) | lsb;
}
static u16 bmp180_read_u16(u8 reg)
{
    u16 msb = i2c_smbus_read_byte_data(bmp180_client, reg);
    u16 lsb = i2c_smbus_read_byte_data(bmp180_client, reg + 1);
    return (msb << 8) | lsb;
}

static long read_unTemperature(void)
{
    
    i2c_smbus_write_byte_data(bmp180_client, BMP180_REG_CONTROL, BMP180_REG_TEMPERATURE);
    msleep(5);
    return bmp180_read_s16(BMP180_REG_RESULT);
}

static long read_unPressure(uint8_t oss)
{
    int wait_ms;
    u8 msb, lsb, xlsb;
    long UP;

    i2c_smbus_write_byte_data(bmp180_client, BMP180_REG_CONTROL,
        BMP180_REG_PRESSURE + (oss << 6));

    switch (oss) {
    case 0: wait_ms = 5;  break;
    case 1: wait_ms = 8;  break;
    case 2: wait_ms = 14; break;
    case 3: wait_ms = 26; break;
    default: return -EINVAL;
    }
    msleep(wait_ms);

    msb = i2c_smbus_read_byte_data(bmp180_client, BMP180_REG_RESULT);
    lsb = i2c_smbus_read_byte_data(bmp180_client, BMP180_REG_RESULT + 1);
    xlsb = i2c_smbus_read_byte_data(bmp180_client, BMP180_REG_RESULT + 2);

    UP = ((msb << 16) | (lsb << 8) | xlsb) >> (8 - oss);
    return UP;
}

static long readTemperature(void)
{
    long UT = read_unTemperature();
    long X1 = ((UT - (long)AC6) * AC5) >> 15;
    long X2 = ((long)MC << 11) / (X1 + MD);
    B5 = X1 + X2;
    return (B5 + 8) >> 4;  // 0.1°C
}

static long readPressure(uint8_t oss)
{
    long UP, B6, X1, X2, X3, B3, p, result;
    unsigned long temp, B4, B7;

    UP = read_unPressure(oss);
    B6 = B5 - 4000;
    X1 = (B2 * ((B6 * B6) >> 12)) >> 11;
    X2 = (AC2 * B6) >> 11;
    X3 = X1 + X2;
    B3 = ((((long)AC1 * 4 + X3) << oss) + 2) >> 4;

    X1 = (AC3 * B6) >> 13;
    temp = (unsigned long)(X3 + 32768);
    B4 = (AC4 * temp) >> 15;
    B7 = ((unsigned long)UP - B3) * (50000 >> oss);

    p = (B7 < 0x80000000) ? (B7 << 1) / B4 : (B7 / B4) << 1;
    X1 = (p >> 8) * (p >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * p) >> 16;
    result = p + ((X1 + X2 + 3791) >> 4);
    
    printk(KERN_INFO "UP: %ld\n", UP);
    printk(KERN_INFO "B4: %ld\n", B4);
    printk(KERN_INFO "B7: %ld\n", B7);
    printk(KERN_INFO "p: %ld\n", p);

    return result;
}

// In giá trị cal
static void bmp180_print_calibration_data(void)
{
    printk(KERN_INFO "BMP180 Calibration Data:\n");
    
    printk(KERN_INFO "AC1: %hd\n", AC1);
    printk(KERN_INFO "AC2: %hd\n", AC2);
    printk(KERN_INFO "AC3: %hd\n", AC3);
    printk(KERN_INFO "B1: %hd\n", B1);
    printk(KERN_INFO "B2: %hd\n", B2);
    printk(KERN_INFO "MB: %hd\n", MB);
    printk(KERN_INFO "MC: %hd\n", MC);
    printk(KERN_INFO "MD: %hd\n", MD);
    printk(KERN_INFO "AC4: %hu\n", AC4);
    printk(KERN_INFO "AC5: %hu\n", AC5);
    printk(KERN_INFO "AC6: %hu\n", AC6);
    printk(KERN_INFO "B5: %ld\n", B5);
}
static int bmp180_open(struct inode *inode, struct file *file)
{
    printk(KERN_INFO "BMP180: Device opened\n");
    return 0;
}

static int bmp180_release(struct inode *inode, struct file *file)
{
    printk(KERN_INFO "BMP180: Device closed\n");
    return 0;
}

static long bmp180_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    long temp;
    struct bmp180_pressure_data data;

    switch (cmd) {
    case IOCTL_READ_TEMP:
        mutex_lock(&bmp180_mutex);
        temp = readTemperature();
        mutex_unlock(&bmp180_mutex);
        if (copy_to_user((long __user *)arg, &temp, sizeof(temp)))
            return -EFAULT;
        break;

    case IOCTL_READ_PRESSURE:
        if (copy_from_user(&data, (struct bmp180_pressure_data __user *)arg, sizeof(data)))
            return -EFAULT;
        if (data.oss > 3)
            return -EINVAL;
        mutex_lock(&bmp180_mutex);
        readTemperature(); // Update B5
        data.pressure = readPressure(data.oss);
        mutex_unlock(&bmp180_mutex);
        if (copy_to_user((struct bmp180_pressure_data __user *)arg, &data, sizeof(data)))
            return -EFAULT;
        break;

    default:
        return -EINVAL;
    }
    return 0;
}

static struct file_operations bmp180_fops = {
    .owner = THIS_MODULE,
    .open = bmp180_open,
    .release = bmp180_release,
    .unlocked_ioctl = bmp180_ioctl,
};

static int bmp180_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)){
        dev_err(&client->dev, "I2C functionality check failed\n");
        return -ENODEV;
    }


    bmp180_client = client;

    major_number = register_chrdev(0, DEVICE_NAME, &bmp180_fops);
    if (major_number < 0)
        return major_number;

    bmp180_class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(bmp180_class)) {
        printk(KERN_ALERT "Failed to register device class\n");
        unregister_chrdev(major_number, DEVICE_NAME);
        return PTR_ERR(bmp180_class);
    }

    bmp180_device = device_create(bmp180_class, NULL, MKDEV(major_number, 0), NULL, DEVICE_NAME);
    if (IS_ERR(bmp180_device)) {
        printk(KERN_ALERT "Failed to create device\n");
        class_destroy(bmp180_class);
        unregister_chrdev(major_number, DEVICE_NAME);
        return PTR_ERR(bmp180_device);
    }

    AC1 = bmp180_read_s16(BMP180_REG_CAL_AC1);
    AC2 = bmp180_read_s16(BMP180_REG_CAL_AC2);
    AC3 = bmp180_read_s16(BMP180_REG_CAL_AC3);
    AC4 = bmp180_read_u16(BMP180_REG_CAL_AC4);
    AC5 = bmp180_read_u16(BMP180_REG_CAL_AC5);
    AC6 = bmp180_read_u16(BMP180_REG_CAL_AC6);
    B1  = bmp180_read_s16(BMP180_REG_CAL_B1);
    B2  = bmp180_read_s16(BMP180_REG_CAL_B2);
    MB  = bmp180_read_s16(BMP180_REG_CAL_MB);
    MC  = bmp180_read_s16(BMP180_REG_CAL_MC);
    MD  = bmp180_read_s16(BMP180_REG_CAL_MD);

    bmp180_print_calibration_data();
    printk(KERN_INFO "BMP180: Device registered with major number %d\n", major_number);
    return 0;
}

static void bmp180_remove(struct i2c_client *client)
{
    device_destroy(bmp180_class, MKDEV(major_number, 0));
    class_unregister(bmp180_class);
    class_destroy(bmp180_class);
    unregister_chrdev(major_number, DEVICE_NAME);
    printk(KERN_INFO "BMP180: Device unregistered\n");
}

static const struct of_device_id bmp180_of_match[] = {
    { .compatible = "thanhtam,bmp180" },
    { }
};
MODULE_DEVICE_TABLE(of, bmp180_of_match);

static const struct i2c_device_id bmp180_id[] = {
    { "bmp180", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, bmp180_id);

static struct i2c_driver bmp180_driver = {
    .driver = {
        .name = DRIVER_NAME,
        .of_match_table = bmp180_of_match,
    },
    .probe    = bmp180_probe,
    .remove   = bmp180_remove,
    .id_table = bmp180_id,
};

// static int __init bmp180_init(void)
// {

//     return i2c_add_driver(&bmp180_driver);
// }
// module_init(bmp180_init);

// static void __exit bmp180_exit(void)
// {
//     i2c_del_driver(&bmp180_driver);
// }
// module_exit(bmp180_exit);
module_i2c_driver(bmp180_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Thanh Tam");
MODULE_DESCRIPTION("BMP180 I2C Driver");
MODULE_VERSION("1.0");

