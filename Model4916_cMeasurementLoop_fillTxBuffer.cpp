/*

Module: Model4916_cMeasurementLoop_fillBuffer.cpp

Function:
    Class for transmitting accumulated measurements.

Copyright:
    See accompanying LICENSE file for copyright and license information.

Author:
    Dhinesh Kumar Pitchai, MCCI Corporation   November 2022

*/

#include <Catena_TxBuffer.h>

#include "Model4916_cMeasurementLoop.h"

#include <arduino_lmic.h>

using namespace McciCatena;
using namespace McciModel4916;

/*

Name:   McciModel4916::cMeasurementLoop::fillTxBuffer()

Function:
    Prepare a messages in a TxBuffer with data from current measurements.

Definition:
    void McciModel4916::cMeasurementLoop::fillTxBuffer(
            cMeasurementLoop::TxBuffer_t& b
            );

Description:
    A format 0x28 message is prepared from the data in the cMeasurementLoop
    object.

*/

void
cMeasurementLoop::fillTxBuffer(
    cMeasurementLoop::TxBuffer_t& b, Measurement const &mData
    )
    {
    gLed.Set(McciCatena::LedPattern::Measuring);


    // initialize the message buffer to an empty state
    b.begin();

    // insert format byte
    b.put(kMessageFormat);

    // the flags in Measurement correspond to the over-the-air flags.
    b.put(std::uint8_t(this->m_data.flags));
    gCatena.SafePrintf("Flag:    %2x\n", std::uint8_t(this->m_data.flags));

    // send Vbat
    if ((this->m_data.flags &  Flags::Vbat) !=  Flags(0))
        {
        float Vbat = mData.Vbat;
        gCatena.SafePrintf("Vbat:    %d mV\n", (int) (Vbat * 1000.0f));
        b.putV(Vbat);
        }

    // print Vbus data
    float Vbus = mData.Vbus;
    gCatena.SafePrintf("Vbus:    %d mV\n", (int) (Vbus * 1000.0f));

    // send boot count
    if ((this->m_data.flags &  Flags::Boot) !=  Flags(0))
        {
        b.putBootCountLsb(this->m_data.BootCount);
        }

    if ((mData.flags & Flags::TH) != Flags(0))
        {
        if (this->m_fSht3x)
            {
            gCatena.SafePrintf(
                    "SHT3x      :  T: %d RH: %d\n",
                    (int) mData.env.TempC,
                    (int) mData.env.Humidity
                    );
            b.putT(mData.env.TempC);
            // no method for 2-byte RH, directly encode it.
            b.put2uf((mData.env.Humidity / 100.0f) * 65535.0f);
            }
        }

    // put co2ppm
    if ((mData.flags & Flags::CO2) != Flags(0))
        {
        gCatena.SafePrintf(
            "SCD30      :  T(C): %c%d.%02d  RH(%%): %d.%02d  CO2(ppm): %d.%02d\n",
            this->ts, this->tint, this->tfrac,
            this->rhint, this->rhfrac,
            this->co2int, this->co2frac
            );

        b.put2u(TxBufferBase_t::f2uflt16(mData.co2ppm.CO2ppm / 40000.0f));
        }

    // put pm and pc data
    if ((mData.flags & Flags::PM) != Flags(0))
        {
        gCatena.SafePrintf(
            "IPS7100    :  PM0.1: %d.%02d  PM0.3: %d.%02d  PM0.5: %d.%02d  PM1.0 %d.%02d  PM2.5 %d.%02d   PM5.0 %d.%02d   PM10 %d.%02d\n",
            (int) mData.particle.Mass[0], this->getDecimal(mData.particle.Mass[0]),
            (int) mData.particle.Mass[1], this->getDecimal(mData.particle.Mass[1]),
            (int) mData.particle.Mass[2], this->getDecimal(mData.particle.Mass[2]),
            (int) mData.particle.Mass[3], this->getDecimal(mData.particle.Mass[3]),
            (int) mData.particle.Mass[4], this->getDecimal(mData.particle.Mass[4]),
            (int) mData.particle.Mass[5], this->getDecimal(mData.particle.Mass[5]),
            (int) mData.particle.Mass[6], this->getDecimal(mData.particle.Mass[6])
            );

        gCatena.SafePrintf(
            "IPS7100    :  PC0.1: %d  PC0.3: %d  PC0.5: %d  PC1.0 %d  PC2.5 %d  PC5.0 %d  PC10 %d\n",
            mData.particle.Count[0],
            mData.particle.Count[1],
            mData.particle.Count[2],
            mData.particle.Count[3],
            mData.particle.Count[4],
            mData.particle.Count[5],
            mData.particle.Count[6]
            );
        }

    // put co
    if ((mData.flags & (Flags::CO | Flags::NO2 | Flags::O3 | Flags::SO2)) != Flags(0))
        {
        gCatena.SafePrintf(
            "ADS131M04  :  CO: %d.%02d  NO2: %d.%02d  O3: %d.%02d  NO2: %d.%02d\n",
            (int) mData.gases.CO, this->getDecimal(mData.gases.CO),
            (int) mData.gases.NO2, this->getDecimal(mData.gases.NO2),
            (int) mData.gases.O3, this->getDecimal(mData.gases.O3),
            (int) mData.gases.SO2, this->getDecimal(mData.gases.SO2)
            );
        }
    gLed.Set(McciCatena::LedPattern::Off);

    // put pm and pc data
    if ((mData.flags & Flags::GPS) != Flags(0))
        {
        gCatena.SafePrintf(
            "SAM-M8Q GPS  :  Latitude(deg): %d.%02d  Longitude(deg): %d.%02d  Unix Time: %d\n",
            (int) mData.position.Latitude, this->getDecimal(mData.position.Latitude),
            (int) mData.position.Longitude, this->getDecimal(mData.position.Longitude),
            (int) mData.position.UnixTime
            );
        }
    }
