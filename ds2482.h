#pragma once

#include <QList>
#include <QString>

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

#define W1_CMD_SEARCH_ROM          0xF0
#define W1_CMD_READ_ROM            0x33
#define W1_CMD_MATCH_ROM           0x55
#define W1_CMD_SKIP_ROM            0xCC
#define W1_CMD_RESUME              0xA5
#define W1_CMD_OVERDRIVE_SKIP_ROM  0x3C
#define W1_CMD_OVERDRIVE_MATCH_ROM 0x69


class DS2482
{
public:
    DS2482();
    ~DS2482();

    int open(QString deviceFile, uint8_t address);
    void close();
    QList<uint64_t> findDevices();

    int select_register(uint8_t read_ptr);
    int reset();
    int wait_w1_idle();

    int set_config(uint8_t config);
    int set_active_pullup(bool activePullup);
    int set_high_speed(bool highSpeed);
    int set_strong_pullup(bool strongPullup);

    int w1_reset();
    int w1_read_bit();
    int w1_write_bit(uint8_t bit);

    int w1_rw_block(uint8_t *buf, int len);

    int w1_write_byte(uint8_t byte);
    int w1_read_byte();

    int w1_triplet(uint8_t *dir, uint8_t *first_bit, uint8_t *second_bit);

private:
    struct w1_search_s;

    int w1_search_lowlevel(w1_search_s *s);
    int fd = -1;
    int config = 0;
};
