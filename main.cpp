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

        ds.set_high_speed(false);
        ds.w1_reset();

//        nanosleep(&sl, NULL);
//        qDebug() << "send overdrive";
        if (ds.w1_write_byte(W1_CMD_OVERDRIVE_SKIP_ROM) != 0)
        {
            fprintf(stderr, "Could not skip overdrive\n");
            return 1;
        }

        ds.set_high_speed(true);
        ds.w1_reset();

        QSet<uint64_t> devices = ds.findDevices().toSet();

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
    }

    ds.close();

    return 0;
}
