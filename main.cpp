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

#define W1_CMD_SEARCH 0xF0

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

int w1_triplet(int fd, uint8_t *dir, uint8_t *first_bit, uint8_t *second_bit)
{
    if (i2c_smbus_write_byte_data(fd, DS2482_CMD_W1_TRIPLET, *dir ? 0xFF : 0) != 0)
    {
        fprintf(stderr, "Could not issue triplet command\n");
        return -1;
    }

    int ret = i2c_smbus_read_byte(fd);
    if (ret < 0)
    {
        fprintf(stderr, "Could not read triplet result\n");
        return -1;
    }

    if (ret & DS2482_STS_SBR_MASK)
    {
        *first_bit = 1;
    } else {
        *first_bit = 0;
    }

    if (ret & DS2482_STS_TSB_MASK)
    {
        *second_bit = 1;
    } else {
        *second_bit = 0;
    }

    if (ret & DS2482_STS_DIR_MASK)
    {
        *dir = 1;
    } else {
        *dir = 0;
    }

    return 0;
}

uint8_t search_serial_number[8];
uint8_t last_discrepancy = 0;

struct w1_search_s {
    void reset() {
        last_device = 0;
        start_search_from = -1;
    }

    w1_search_s() {
        reset();
    }

    uint64_t last_device;
    int start_search_from = -1;
};

// d7b2ad2d

// d7b2af2d

// d7be6b2d

int w1_search(int fd, w1_search_s *s)
{
    if (w1_reset(fd) != 0)
    {
        fprintf(stderr, "Could not reset w1 bus\n");
        s->reset();
        return -1;
    }

    if (w1_write_byte(fd, W1_CMD_SEARCH) != 0)
    {
        fprintf(stderr, "Could not write search command\n");
        s->reset();
        return -1;
    }

    int cur_bit = 0;
    uint8_t dir;
    uint8_t first_bit, second_bit;

    int last_zero = -1;

    do
    {
        if (cur_bit < s->start_search_from)
        {
            dir = s->last_device & (1 << cur_bit) ? 1 : 0;
            qDebug() << "choosing" << dir << "at bit" << cur_bit;
        }
        else if (cur_bit == s->start_search_from)
        {
            // we are at the point of the last branch where we chose 0, now we choose 1
            dir = 1;
        } else {
            dir = 0;
        }

        int chosen_dir = 1;

        if (w1_triplet(fd, &dir, &first_bit, &second_bit) != 0)
        {
            fprintf(stderr, "Could not triplet on bit %d\n", cur_bit);
            return -1;
        }
//        if (cur_bit == s->start_search_from)
//        {
//            dir = 1;
//        }

        qDebug() << "cur_bit" << cur_bit << "sending triplet" << chosen_dir << "got dir" << dir << "bits" << first_bit << second_bit;

        if (first_bit == 1 && second_bit == 1)
        {
            // no devices found
            // reset search
            s->reset();
            return 0;
        }

        if (first_bit == 0 && second_bit == 0 && dir == 0)
        {
            // discrepancy found
            last_zero = cur_bit;
        }

        if (dir == 1)
        {
            s->last_device |= (1 << cur_bit);
        } else {
            s->last_device &= ~(1 << cur_bit);
        }

        cur_bit++;
    } while (cur_bit < 64);

    if (cur_bit == 64)
    {
        // successful search
        s->start_search_from = last_zero;

        return 1;
    }

    return 0;
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

    w1_search_s s;

    int ret = w1_search(fd, &s);
    if (ret == 0)
    {
        qDebug() << "no devices found";
    } else {
        qDebug() << "device found" << QString("%1").arg(s.last_device, 0, 16);
    }

    ret = w1_search(fd, &s);
    if (ret == 0)
    {
        qDebug() << "no devices found";
    } else {
        qDebug() << "device found" << QString("%1").arg(s.last_device, 0, 16);
    }
    close(fd);


    return 0;
}
