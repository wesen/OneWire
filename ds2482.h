#pragma once

#include <QList>
#include <QString>

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



/*!
 * \class DS2482
 *
 * \brief Simple userland interface to the DS2482 i2c/one wire master
 */
class DS2482
{
public:
    DS2482();
    ~DS2482();

    enum ds2482_reg_t {
        DS2482_REG_STS = 0xF0,
        DS2482_REG_DATA = 0xE1,
        DS2482_REG_CFG = 0xC3
    };

    enum ds2482_cmd_t {
        DS2482_CMD_RESET          = 0xF0,
        DS2482_CMD_SET_READ_PTR   = 0xE1,
        DS2482_CMD_CHANNEL_SELECT = 0xC3,
        DS2482_CMD_WRITE_CONFIG   = 0xD1,
        DS2482_CMD_W1_RESET       = 0xB4,
        DS2482_CMD_W1_SINGLE_BIT  = 0x87,
        DS2482_CMD_W1_WRITE_BYTE  = 0xA5,
        DS2482_CMD_W1_READ_BYTE   = 0x96,
        DS2482_CMD_W1_TRIPLET     = 0x78
    };

    enum w1_cmd_t {
        W1_CMD_SEARCH_ROM =          0xF0,
        W1_CMD_READ_ROM =            0x33,
        W1_CMD_MATCH_ROM =           0x55,
        W1_CMD_SKIP_ROM =            0xCC,
        W1_CMD_RESUME =              0xA5,
        W1_CMD_OVERDRIVE_SKIP_ROM =  0x3C,
        W1_CMD_OVERDRIVE_MATCH_ROM = 0x69
    };

    enum ds2431_cmd_t {
        DS2431_CMD_WRITE_SCRATCHPAD = 0x0F,
        DS2431_CMD_READ_SCRATCHPAD  = 0xAA,
        DS2431_CMD_COPY_SCRATCHPAD  = 0x55,
        DS2431_CMD_READ_MEMORY      = 0xF0
    };

    /*!
     * \brief open - opens the i2c bus and selects the ds2482 slave
     * \param deviceFile - i2c device file, for example "/dev/i2c-2"
     * \param address - i2c slave address of DS2482
     * \return 0 on success, -1 on failure
     */
    int open(QString deviceFile, uint8_t address);
    /*!
     * \brief close the opened i2c device (if open)
     */
    void close();
    /*!
     * \brief findDevices - scans the 1-wire bus and returns found device ids
     * \return list of found device ids
     */
    QList<uint64_t> findDevices();

    /*!
     * \brief select_register - sets the DS2482 read pointer
     * \param read_ptr
     * \return 0 on success, -1 on failure
     */
    int select_register(ds2482_reg_t read_ptr);
    int reset();
    int wait_w1_idle();

    typedef uint8_t ds2482_config_t;
    typedef uint8_t bit_t;

    int set_config(ds2482_config_t config);
    int set_active_pullup(bool activePullup);
    int set_high_speed(bool highSpeed);
    int set_strong_pullup(bool strongPullup);

    /*!
     * \brief w1_reset
     * \return 1 if presence pulse was detected, 0 when no presence pulse was detected, -1 on error
     */
    int w1_reset();
    int w1_read_bit();
    int w1_write_bit(bit_t bit);

    int w1_rw_block(uint8_t *buf, int len);
    int w1_write_block(uint8_t *buf, int len);

    /*!
     * \brief w1_write_byte
     * \param byte - byte to be written
     * \return 0 on success, -1 on failure
     */
    int w1_write_byte(uint8_t byte);
    int w1_read_byte();

    int w1_match_rom(uint64_t device);
    int w1_overdrive_match_rom(uint64_t device);

    int w1_resume();

    int w1_triplet(bit_t *dir, bit_t *first_bit, bit_t *second_bit);

    static bool w1_check_rom_crc(uint64_t dev);

private:
    struct w1_search_s;

    int w1_search_lowlevel(w1_search_s *s);
    int fd = -1;
    int config = 0;
};
