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

    for (;;)
    {
        QList<uint64_t> devices = ds.findDevices();
        foreach (uint64_t dev, devices)
        {
            qDebug() << "found" << QString("%1").arg(dev, 0, 16);
        }

        struct timespec sl = { .tv_sec = 0, .tv_nsec = 500 * 1000 };


        nanosleep(&sl, NULL);
    }

    ds.close();

    return 0;
}
