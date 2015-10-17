#include <QCoreApplication>
#include <QDebug>

#include <stdio.h>

#include <inttypes.h>
#include <linux/i2c-dev-user.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#define ADDRESS 0x18

#define DS2482_CMD_RESET          0xF0
#define DS2482_CMD_SET_READ_PTR   0xE1
#define DS2482_CMD_CHANNEL_SELECT 0xC3
#define DS2482_CMD_WRITE_CONFIG   0xD2
#define DS2482_CMD_W1_RESET       0xB4
#define DS2482_CMD_W1_SINGLE_BIT  0x87
#define DS2482_CMD_W1_WRITE_BYTE  0xA5
#define DS2482_CMD_W1_READ_BYTE   0x96
#define DS2482_CMD_W1_TRIPLET     0x78

#define DS2482_REG_STS  0xF0
#define DS2482_REG_DATA 0xE1
#define DS2482_REG_CFG  0xC3

#define DS2482_STS_1WB_MASK 1
#define DS2482_STS_PPD_MASK (1 << 1)
#define DS2482_STS_SD_MASK  (1 << 2)
#define DS2482_STS_LL_MASK  (1 << 3)
#define DS2482_STS_RST_MASK (1 << 4)
#define DS2482_STS_SBR_MASK (1 << 5)
#define DS2482_STS_TSB_MASK (1 << 6)
#define DS2482_STS_DIR_MASK (1 << 7)

#define DS2482_REG_APU_MASK 1
#define DS2482_REG_SPU_MASK (1 << 2)
#define DS2482_REG_1WS_MASK (1 << 3)

#define DS2482_IDLE_TIMEOUT 100

QString ds2482_sts_reg_to_string(int reg)
{
    QStringList bits;

    if (reg & DS2482_STS_1WB_MASK) { bits << "1WB"; }
    if (reg & DS2482_STS_PPD_MASK) { bits << "PPD"; }
    if (reg & DS2482_STS_SD_MASK) { bits << "SD"; }
    if (reg & DS2482_STS_LL_MASK) { bits << "LL"; }
    if (reg & DS2482_STS_RST_MASK) { bits << "RST"; }
    if (reg & DS2482_STS_SBR_MASK) { bits << "SBR"; }
    if (reg & DS2482_STS_TSB_MASK) { bits << "TSB"; }
    if (reg & DS2482_STS_DIR_MASK) { bits << "DIR"; }

    return "<" + bits.join(",") + ">";
}

int ds2482_select_register(int fd, uint8_t read_ptr)
{
    if (i2c_smbus_write_byte_data(fd, DS2482_CMD_SET_READ_PTR, read_ptr) < 0)
    {
        fprintf(stderr, "Could not set read_ptr %d\n", read_ptr);
        return -1;
    }

    printf("select register: %x\n", read_ptr);

    return 0;
}

int ds2482_wait_w1_idle(int fd)
{
    if (ds2482_select_register(fd, DS2482_REG_STS) == 0)
    {
        int tmp = 0;
        int retries = 0;
        do {
            tmp = i2c_smbus_read_byte(fd);
            printf("got status byte: %x\n", tmp);
        } while ((tmp >= 0) && (tmp & DS2482_STS_1WB_MASK)
                 && (++retries < DS2482_IDLE_TIMEOUT));

        if (retries == DS2482_IDLE_TIMEOUT)
        {
            fprintf(stderr, "Timeout while waiting for bus to be idle\n");
            return -1;
        }
    }

    return 0;
}

int ds2482_reset(int fd)
{
    if (i2c_smbus_write_byte(fd, DS2482_CMD_RESET) != 0)
    {
        fprintf(stderr, "Could not send device reset command\n");
        return -1;
    }
    int ret = i2c_smbus_read_byte(fd);
    printf("got device reset result: %x\n", ret);

    return ret;
}

int w1_reset(int fd)
{
    if (ds2482_wait_w1_idle(fd) != 0)
    {
        fprintf(stderr, "Could not wait for W1 bus idle\n");
        return -1;
    }

    if (i2c_smbus_write_byte(fd, DS2482_CMD_W1_RESET) != 0)
    {
        fprintf(stderr, "Could not send w1 reset command\n");
        return -1;
    }

    if (ds2482_wait_w1_idle(fd) != 0)
    {
        fprintf(stderr, "Could not wait for w1 bus idle\n");
        return -1;
    }

    int ret = i2c_smbus_read_byte(fd);
    if (ret & DS2482_STS_PPD_MASK)
    {
        printf("presence pulse detected\n");
    } else {
        printf("no presence pulse detected\n");
    }

    return 0;
}

int w1_write_bit(int fd, uint8_t bit)
{
    if (ds2482_wait_w1_idle(fd) != 0)
    {
        fprintf(stderr, "Could not wait for W1 bus idle\n");
        return -1;
    }

    if (i2c_smbus_write_byte_data(fd, DS2482_CMD_W1_SINGLE_BIT, bit == 0 ? 0x7F : 0xFF) != 0)
    {
        fprintf(stderr, "Could not write W1 single bit\n");
        return -1;
    }

    // wait for idle so following commands don't have to
    if (ds2482_wait_w1_idle(fd) != 0)
    {
        fprintf(stderr, "Could not wait for W1 bus idle\n");
        return -1;
    }

    return 0;
}

int w1_read_bit(int fd)
{
    if (w1_write_bit(fd, 1) != 0)
    {
        fprintf(stderr, "Could not write bit to prepare for read\n");
        return -1;
    }

    if (ds2482_select_register(fd, DS2482_REG_STS))
    {
        fprintf(stderr, "Could not switch to status register\n");
        return -1;
    }

    int ret = i2c_smbus_read_byte(fd);
    if (ret < 0)
    {
        fprintf(stderr, "Could not read status byte\n");
        return -1;
    }

    if (ret & DS2482_STS_SBR_MASK)
    {
        return 1;
    } else {
        return 0;
    }
}

int w1_write_byte(int fd, uint8_t byte)
{
    if (ds2482_wait_w1_idle(fd) != 0)
    {
        fprintf(stderr, "Could not wait for W1 bus idle\n");
        return -1;
    }

    if (i2c_smbus_write_byte_data(fd, DS2482_CMD_W1_WRITE_BYTE, byte) != 0)
    {
        fprintf(stderr, "Could not write W1 byte\n");
        return -1;
    }

    // wait for idle so following commands don't have to
    if (ds2482_wait_w1_idle(fd) != 0)
    {
        fprintf(stderr, "Could not wait for W1 bus idle\n");
        return -1;
    }

    return 0;
}

int w1_read_byte(int fd)
{
    if (ds2482_wait_w1_idle(fd) != 0)
    {
        fprintf(stderr, "Could not wait for W1 bus idle\n");
        return -1;
    }

    if (i2c_smbus_write_byte(fd, DS2482_CMD_W1_READ_BYTE) != 0)
    {
        fprintf(stderr, "Could not read W1 byte\n");
        return -1;
    }

    if (ds2482_select_register(fd, DS2482_REG_DATA))
    {
        fprintf(stderr, "Could not switch to data register\n");
        return -1;
    }

    int ret = i2c_smbus_read_byte(fd);
    if (ret < 0)
    {
        fprintf(stderr, "Could not read data byte\n");
        return -1;
    }

    return ret;
}

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    int fd = open("/dev/i2c-2", O_RDWR);
    if (fd < 0)
    {
        fprintf(stderr, "Could not open i2c device\n");
        return 1;
    }

    if (ioctl(fd, I2C_SLAVE, ADDRESS) < 0)
    {
        fprintf(stderr, "Failed to set slave address: %m\n");
        return 1;
    }
    qDebug() << "opened i2c device and set adderess";

    ds2482_reset(fd);
    w1_reset(fd);

    close(fd);


    return 0;
}
