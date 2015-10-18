#include <QCoreApplication>
#include <QDebug>

#include <stdio.h>
#include <time.h>

#include <inttypes.h>
#include <linux/i2c-dev-user.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#define ADDRESS 0x18

#include "ds2482.h"

QSet<uint64_t> highspeedScan(DS2482 &ds)
{
    ds.set_high_speed(false);
    ds.w1_reset();

    //        nanosleep(&sl, NULL);
    //        qDebug() << "send overdrive";
    if (ds.w1_write_byte(DS2482::W1_CMD_OVERDRIVE_SKIP_ROM) != 0)
    {
        fprintf(stderr, "Could not skip overdrive\n");
        return QSet<uint64_t>();
    }

    ds.set_high_speed(true);
    ds.w1_reset();

    QSet<uint64_t> devices = ds.findDevices().toSet();

    return devices;
}

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    DS2482 ds;
    if (ds.open("/dev/i2c-2", 0x18) != 0)
    {
        fprintf(stderr, "Could not open i2c device\n");
        return 1;
    }

    ds.set_active_pullup(true);

    QSet<uint64_t> prevDevices;

    for (;;)
    {
        static struct timespec sl = {
            0,
            100 * 1000 * 1000
        };

        qDebug() << "--";

        QSet<uint64_t> devices = highspeedScan(ds);

        for (int i = 0; i < 1; i++) {
//            devices.unite(highspeedScan(ds));
//            nanosleep(&sl, NULL);
        }

        QSet<uint64_t> newDevices = devices - prevDevices;
        foreach (uint64_t dev, newDevices)
        {
            qDebug() << "found" << QString("%1").arg(dev, 0, 16);
        }

        QSet<uint64_t> removedDevices = prevDevices - devices;
        foreach (uint64_t dev, removedDevices)
        {
            qDebug() << "removed" << QString("%1").arg(dev, 0, 16);
        }

        prevDevices = devices;

        nanosleep(&sl, NULL);

        foreach (uint64_t dev, prevDevices)
        {
            if (ds.w1_match_rom(dev) != 0)
            {
                fprintf(stderr, "Could not match rom: %llx\n", dev);
                continue;
            }

        for (int i = 0; i < 8; i++)
        {
            ds.w1_resume();

            {
                uint8_t data[11] = {
                    DS2482::DS2431_CMD_WRITE_SCRATCHPAD,
                    0x00, 0x00,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08
                };
                ds.w1_write_block(data, 3);
                ds.w1_write_block(data + 3, 8);

                uint16_t crc = DS2482::w1_compute_data_crc(data, 11);
                printf("CRC: %x\n", crc);
            }

//             read crc
            uint8_t crc[4];
            ds.w1_read_block(crc, 4);

            printf("CRC: %x %x %x %x\n", crc[0], crc[1], crc[2], crc[3]);

            ds.w1_resume();

            ds.w1_write_byte(DS2482::DS2431_CMD_READ_SCRATCHPAD);
            {
                uint8_t data[16] = { 0 };
                ds.w1_read_block(data + 1, 15);
                data[0] = DS2482::DS2431_CMD_READ_SCRATCHPAD;
                for (int i = 0; i < 16; i++)
                {
                    printf("%d: %x\n", i, data[i]);
                }

                uint16_t crc = DS2482::w1_compute_data_crc(data, 12);
                printf("CRC: %x\n", crc);
            }
        }
        }

        nanosleep(&sl, NULL);
        break;
    }

    ds.close();

    return 0;
}
