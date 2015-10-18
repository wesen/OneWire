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
        if (w1_check_rom_crc(s->last_device))
        {
            // successful search
            s->start_search_from = last_zero;
            if (last_zero == -1)
            {
                return 2;
            } else {
                return 1;
            }
        } else {
            fprintf(stderr, "Invalid crc\n");
            s->reset();
            return -1;
        }
    }

    return 0;
}

int DS2482::select_register(ds2482_reg_t read_ptr)
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

int DS2482::w1_read_block(uint8_t *buf, int len)
{
    for (int i = 0; i < len; i++)
    {
        int ret = w1_read_byte();
        if (ret < 0)
        {
            return ret;
        }

        buf[i] = ret;
    }

    return len;
}

int DS2482::w1_write_block(uint8_t *buf, int len)
{
    for (int i = 0; i < len; i++)
    {
        int ret = w1_write_byte(buf[i]);
        if (ret < 0)
        {
            return ret;
        }
    }

    return 0;
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

    if (wait_w1_idle() != 0)
    {
        fprintf(stderr, "Could not wait for W1 bus idle\n");
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

int DS2482::w1_match_rom(uint64_t device)
{
    if (w1_reset() < 0)
    {
        fprintf(stderr, "Could not reset the w1 bus\n");
        return -1;
    }

    uint8_t data[1 + 8];
    data[0] = W1_CMD_MATCH_ROM;
    for (int i = 0; i < 8; i++)
    {
        data[i + 1] = device & 0xFF;
        device >>= 8;
    }

    if (w1_write_block(data, 9) != 0)
    {
        fprintf(stderr, "Could not write match rom data to bus\n");
        return -1;
    }

    return 0;
}

int DS2482::w1_resume()
{
    if (w1_reset() < 0)
    {
        fprintf(stderr, "Could not reset the w1 bus\n");
        return -1;
    }

    if (w1_write_byte(W1_CMD_RESUME) != 0)
    {
        return -1;
    }

    return 0;
}

int DS2482::w1_overdrive_match_rom(uint64_t device)
{
    if (w1_reset() < 0)
    {
        fprintf(stderr, "Could not reset the w1 bus\n");
        return -1;
    }

    if (w1_write_byte(W1_CMD_OVERDRIVE_MATCH_ROM) != 0)
    {
        fprintf(stderr, "Could not write OVERDRIVE MATCH ROM command\n");
        return -1;
    }

    set_high_speed(true);

    uint8_t data[8];
    for (int i = 0; i < 8; i++)
    {
        data[i] = device & 0xFF;
        device >>= 8;
    }

    return w1_write_block(data, 8);
}

bool DS2482::w1_check_rom_crc(uint64_t dev)
{
    static uint8_t crcLookup[256] = {
      0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
      157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
      35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
      190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
      70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
      219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
      101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
      248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
      140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
      17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
      175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
      50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
      202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
      87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
      233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
      116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53
    };

    uint8_t _crc = 0;
    for (int i = 0; i < 8; i++) {
      _crc = crcLookup[_crc ^ (dev & 0xFF)];
      dev >>= 8;
    }

    return _crc == 0;
}
