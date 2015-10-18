#include "ds2482.h"

#include <stdio.h>

#include <inttypes.h>
#include <linux/i2c-dev-user.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include <QDebug>

DS2482::DS2482()
{

}

DS2482::~DS2482()
{
    if (fd != -1)
    {
        close();
    }
}

void DS2482::close()
{
    if (fd != -1)
    {
        ::close(fd);
        fd = -1;
    }
}

int DS2482::open(QString deviceFile, uint8_t address)
{
    fd = ::open(deviceFile.toLatin1().constData(), O_RDWR);
    if (fd < 0)
    {
        fprintf(stderr, "Could not open i2c device\n");
        return -1;
    }

    if (ioctl(fd, I2C_SLAVE, address) < 0)
    {
        fprintf(stderr, "Failed to set slave address: %m\n");
        close();
        return -1;
    }

    int ret = reset();
    if (ret < 0)
    {
        fprintf(stderr, "Could not reset ds2482\n");
        close();
        return -1;
    }

    return 0;
}

struct DS2482::w1_search_s {
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

QList<uint64_t> DS2482::findDevices()
{
    w1_search_s s;
    QList<uint64_t> result;

    int ret = 0;
    for (;;) {
        ret = w1_search_lowlevel(&s);

        if (ret != 0)
        {
            result << s.last_device;
        }

        if (ret == 0 || ret == 2)
        {
            break;
        }
    }

    return result;

}

int DS2482::w1_search_lowlevel(w1_search_s *s)
{
    int ret = w1_reset();
    if (ret < 0)
    {
        fprintf(stderr, "Could not reset w1 bus\n");
        s->reset();
        return -1;
    }
    if (ret == 0)
    {
        // no presence pulse
        return 0;
    }

    if (w1_write_byte(W1_CMD_SEARCH_ROM) != 0)
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
        }
        else if (cur_bit == s->start_search_from)
        {
            // we are at the point of the last branch where we chose 0, now we choose 1
            dir = 1;
        } else {
            dir = 0;
        }

        if (w1_triplet(&dir, &first_bit, &second_bit) != 0)
        {
            fprintf(stderr, "Could not triplet on bit %d\n", cur_bit);
            return -1;
        }
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
            s->last_device |= (1ULL << cur_bit);
        } else {
            s->last_device &= ~(1ULL << cur_bit);
        }

        cur_bit++;
    } while (cur_bit < 64);

    if (cur_bit == 64)
    {
        // successful search
        s->start_search_from = last_zero;
        if (last_zero == -1)
        {
            return 2;
        } else {
            return 1;
        }
    }

    return 0;
}

int DS2482::select_register(uint8_t read_ptr)
{
    if (i2c_smbus_write_byte_data(fd, DS2482_CMD_SET_READ_PTR, read_ptr) < 0)
    {
        fprintf(stderr, "Could not set read_ptr %d\n", read_ptr);
        return -1;
    }

    return 0;
}

int DS2482::reset()
{
    if (i2c_smbus_write_byte(fd, DS2482_CMD_RESET) != 0)
    {
        fprintf(stderr, "Could not send device reset command\n");
        return -1;
    }
    int ret = i2c_smbus_read_byte(fd);

    config = 0;

    return ret;

}

int DS2482::wait_w1_idle()
{
    if (select_register(DS2482_REG_STS) == 0)
    {
        int tmp = 0;
        int retries = 0;
        do {
            tmp = i2c_smbus_read_byte(fd);
            if (tmp & DS2482_STS_SD_MASK)
            {
                qDebug() << "bus shorted";
            }
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

int DS2482::set_config(uint8_t _config)
{
    _config &= 0x0F;
    _config = (~_config << 4) | _config;

    if (i2c_smbus_write_byte_data(fd, DS2482_CMD_WRITE_CONFIG, _config) != 0)
    {
        fprintf(stderr, "Could not write config byte\n");
        return -1;
    }

    config = _config;

    return 0;
}

int DS2482::set_active_pullup(bool activePullup)
{
    uint8_t _config = config;

    if (activePullup)
    {
        _config |= DS2482_REG_APU_MASK;
    } else {
        _config &= ~DS2482_REG_APU_MASK;
    }

    return set_config(_config);
}

int DS2482::set_high_speed(bool highSpeed)
{
    uint8_t _config = config;

    if (highSpeed)
    {
        _config |= DS2482_REG_1WS_MASK;
    } else {
        _config &= ~DS2482_REG_1WS_MASK;
    }

    return set_config(_config);
}

int DS2482::set_strong_pullup(bool strongPullup)
{
    uint8_t _config = config;

    if (strongPullup)
    {
        _config |= DS2482_REG_SPU_MASK;
    } else {
        _config &= ~DS2482_REG_SPU_MASK;
    }

    return set_config(_config);
}

int DS2482::w1_reset()
{
    if (wait_w1_idle() != 0)
    {
        fprintf(stderr, "Could not wait for W1 bus idle\n");
        return -1;
    }

    if (i2c_smbus_write_byte(fd, DS2482_CMD_W1_RESET) != 0)
    {
        fprintf(stderr, "Could not send w1 reset command\n");
        return -1;
    }

    if (wait_w1_idle() != 0)
    {
        fprintf(stderr, "Could not wait for w1 bus idle\n");
        return -1;
    }

    int ret = i2c_smbus_read_byte(fd);
    if (ret & DS2482_STS_PPD_MASK)
    {
        return 1;
    } else {
        return 0;
    }

    return 0;


}

int DS2482::w1_read_bit()
{
    if (w1_write_bit(1) != 0)
    {
        fprintf(stderr, "Could not write bit to prepare for read\n");
        return -1;
    }

    if (select_register(DS2482_REG_STS))
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

int DS2482::w1_write_bit(uint8_t bit)
{
    if (wait_w1_idle() != 0)
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
    if (wait_w1_idle() != 0)
    {
        fprintf(stderr, "Could not wait for W1 bus idle\n");
        return -1;
    }

    return 0;

}

int DS2482::w1_write_byte(uint8_t byte)
{
    if (wait_w1_idle() != 0)
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
    if (wait_w1_idle() != 0)
    {
        fprintf(stderr, "Could not wait for W1 bus idle\n");
        return -1;
    }

    return 0;
}

int DS2482::w1_rw_block(uint8_t *buf, int len)
{
    for (int i = 0; i < len; i++)
    {
        int ret = w1_write_byte(buf[1]);
        if (ret < 0)
        {
            return ret;
        }

        if (select_register(DS2482_REG_DATA))
        {
            fprintf(stderr, "Could not switch to data register\n");
            return -1;
        }

        ret = i2c_smbus_read_byte(fd);
        if (ret < 0)
        {
            fprintf(stderr, "Could not read data byte\n");
            return -1;
        }

        buf[i] = ret;
    }

    return len;
}

int DS2482::w1_read_byte()
{
    if (wait_w1_idle() != 0)
    {
        fprintf(stderr, "Could not wait for W1 bus idle\n");
        return -1;
    }

    if (i2c_smbus_write_byte(fd, DS2482_CMD_W1_READ_BYTE) != 0)
    {
        fprintf(stderr, "Could not read W1 byte\n");
        return -1;
    }

    if (select_register(DS2482_REG_DATA))
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

int DS2482::w1_triplet(uint8_t *dir, uint8_t *first_bit, uint8_t *second_bit)
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


    if (ret & DS2482_STS_SD_MASK)
    {
        qDebug() << "bus shorted";
    }

    return 0;
}
