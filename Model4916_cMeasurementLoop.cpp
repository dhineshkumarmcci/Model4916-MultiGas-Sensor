/*

Module: Model4916_cMeasurementLoop.cpp

Function:
    Class for transmitting accumulated measurements.

Copyright:
    See accompanying LICENSE file for copyright and license information.

Author:
    Dhinesh Kumar Pitchai, MCCI Corporation   November 2022

*/

#include "Model4916_cMeasurementLoop.h"

#include <arduino_lmic.h>
#include <Model4916-MultiGas-Sensor.h>
#include <stdint.h>

using namespace McciModel4916;
using namespace McciCatena;

extern cMeasurementLoop gMeasurementLoop;

/* instantiate SPI */
/*SPIClass gSPI2(
		Catena::PIN_SPI2_MOSI,
		Catena::PIN_SPI2_MISO,
		Catena::PIN_SPI2_SCK
		);*/

/****************************************************************************\
|
|   An object to represent the uplink activity
|
\****************************************************************************/

void cMeasurementLoop::begin()
    {
    // register for polling.
    if (! this->m_registered)
        {
        this->m_registered = true;

        gCatena.registerObject(this);

        this->m_UplinkTimer.begin(this->m_txCycleSec * 1000);
        }

    Wire.begin();

    this->m_bme680.begin(BME680_I2C_ADDR_SECONDARY, Wire);
    this->m_fBme680 = true;
    /* if (this->m_bme680.begin(BME680_I2C_ADDR_SECONDARY, Wire))
        {
        this->m_fBme680 = true;
        }
    else
        {
        this->m_fBme680 = false;
        gCatena.SafePrintf("No BME680 found: check wiring\n");
        }*/

    if (this->m_Sht.begin())
        {
        this->m_fSht3x = true;
        }
    else
        {
        this->m_fSht3x = false;
        gCatena.SafePrintf("No SHT3x found: check wiring\n");
        }

    if (! m_Scd.begin())
        {
        this->m_fScd30 = false;

        gCatena.SafePrintf("No SCD30 found! Begin failed: %s(%u)\n",
                m_Scd.getLastErrorName(),
                unsigned(m_Scd.getLastError())
                );
        }
    else
        {
        this->m_fScd30 = true;
        this->m_fSleepScd30 = false;
        this->printSCDinfo();
        }

    if (this->m_Ips.begin())
        {
        this->m_fIps7100 = true;
        }
    else
        {
        this->m_fIps7100 = false;
        gCatena.SafePrintf("No IPS-7100 found: check wiring\n");
        }

    if (this->m_Ads.begin(&gSPI2))
        {
        this->m_fAds131m04 = true;
        }
    else
        {
        this->m_fAds131m04 = false;
        gCatena.SafePrintf("No ADS131M04 found: check wiring\n");
        }

    if (this->m_Gps.begin())
        {
        this->m_GpsSamM8q = true;
        this->configGps();
        }
    else
        {
        this->m_GpsSamM8q = false;
        gCatena.SafePrintf("No SAM-M8Q GPS found: check wiring\n");
        }

    // start (or restart) the FSM.
    if (! this->m_running)
        {
        this->m_exit = false;
        this->m_fsm.init(*this, &cMeasurementLoop::fsmDispatch);
        }
    }

void cMeasurementLoop::end()
    {
    if (this->m_running)
        {
        this->m_exit = true;
        this->m_fsm.eval();
        }
    }

void cMeasurementLoop::requestActive(bool fEnable)
    {
    if (fEnable)
        this->m_rqActive = true;
    else
        this->m_rqInactive = true;

    this->m_fsm.eval();
    }

cMeasurementLoop::State
cMeasurementLoop::fsmDispatch(
    cMeasurementLoop::State currentState,
    bool fEntry
    )
    {
    State newState = State::stNoChange;

    if (fEntry && this->isTraceEnabled(this->DebugFlags::kTrace))
        {
        gCatena.SafePrintf("cMeasurementLoop::fsmDispatch: enter %s\n",
                this->getStateName(currentState)
                );
        }

    switch (currentState)
        {
    case State::stInitial:
        newState = State::stInactive;
        this->resetMeasurements();
        break;

    case State::stInactive:
        if (fEntry)
            {
            // turn off anything that should be off while idling.
            }
        if (this->m_rqActive)
            {
            // when going active manually, start the measurement
            // cycle immediately.
            this->m_rqActive = this->m_rqInactive = false;
            this->m_active = true;
            this->m_UplinkTimer.retrigger();
            newState = State::stWarmup;
            }
        break;

    case State::stSleeping:
        if (fEntry)
            {
            // set the LEDs to flash accordingly.
            gLed.Set(McciCatena::LedPattern::Sleeping);
            }

        if (this->m_rqInactive)
            {
            this->m_rqActive = this->m_rqInactive = false;
            this->m_active = false;
            newState = State::stInactive;
            }
        else if (this->m_UplinkTimer.isready())
            newState = State::stMeasure;
        else if (this->m_UplinkTimer.getRemaining() > 1500)
            {
            this->m_fSleepScd30 = true;
            this->sleep();
            }
        break;

      // get some data. This is only called while booting up.
	     case State::stWarmup:
	         if (fEntry)
	             {
	             //start the timer
	             this->setTimer(5 * 1000);
	             }
	         if (this->timedOut())
	             newState = State::stMeasure;
	         break;

    // fill in the measurement
    case State::stMeasure:
			if (fEntry)
            {
            this->updateSynchronousMeasurements();
            }

            newState = State::stTransmit;
        break;

    case State::stTransmit:
        if (fEntry)
            {
            TxBuffer_t b;
            this->fillTxBuffer(b, this->m_data);

            this->m_FileTxBuffer.begin();
            for (auto i = 0; i < b.getn(); ++i)
                this->m_FileTxBuffer.put(b.getbase()[i]);

            this->resetMeasurements();
            this->startTransmission(b);
            }
        if (! gLoRaWAN.IsProvisioned())
            {
            newState = State::stFinal;
            }
        if (this->txComplete())
            {
            newState = State::stSleeping;

            // calculate the new sleep interval.
            this->updateTxCycleTime();
            }
        break;

    case State::stFinal:
        break;

    default:
        break;
        }

    return newState;
    }

/****************************************************************************\
|
|   Take a measurement
|
\****************************************************************************/

void cMeasurementLoop::resetMeasurements()
    {
    memset((void *) &this->m_data, 0, sizeof(this->m_data));
    this->m_data.flags = Flags(0);
    }

void cMeasurementLoop::updateScd30Measurements()
    {
    if (this->m_fScd30)
        {
        bool fError;
        if (this->m_Scd.queryReady(fError))
            {
            this->m_measurement_valid = this->m_Scd.readMeasurement();
            if ((! this->m_measurement_valid) && gLog.isEnabled(gLog.kError))
                {
                gLog.printf(gLog.kError, "SCD30 measurement failed: error %s(%u)\n",
                        this->m_Scd.getLastErrorName(),
                        unsigned(this->m_Scd.getLastError())
                        );
                }
            }
        else if (fError)
            {
            if (gLog.isEnabled(gLog.DebugFlags::kError))
                gLog.printf(
                    gLog.kAlways,
                    "SCD30 queryReady failed: status %s(%u)\n",
                    this->m_Scd.getLastErrorName(),
                    unsigned(this->m_Scd.getLastError())
                    );
            }
        }

    if (this->m_fScd30 && this->m_measurement_valid)
        {
        auto const m = this->m_Scd.getMeasurement();
        // temperature is 2 bytes from -163.840 to +163.835 degrees C
        // pressure is 4 bytes, first signed units, then scale.
        if (gLog.isEnabled(gLog.kInfo))
            {
            this->ts = ' ';
            this->t100 = std::int32_t(m.Temperature * 100.0f + 0.5f);
            if (m.Temperature < 0) {
                this->ts = '-';
                this->t100 = -this->t100;
                }
            this->tint = this->t100 / 100;
            this->tfrac = this->t100 - (tint * 100);

            this->rh100 = std::int32_t(m.RelativeHumidity * 100.0f + 0.5f);
            this->rhint = this->rh100 / 100;
            this->rhfrac = this->rh100 - (this->rhint * 100);

            this->co2_100 = std::int32_t(m.CO2ppm * 100.0f + 0.5f);
            this->co2int = this->co2_100 / 100;
            this->co2frac = this->co2_100 - (this->co2int * 100);
            }

        this->m_data.co2ppm.CO2ppm = m.CO2ppm;
        }
    }

void cMeasurementLoop::updateSynchronousMeasurements()
    {
    this->m_data.Vbat = gCatena.ReadVbat();
    this->m_data.flags |= Flags::Vbat;

    if (gCatena.getBootCount(this->m_data.BootCount))
        {
        this->m_data.flags |= Flags::Boot;
        }

    if (this->m_fSht3x)
        {
        cSHT3x::Measurements m;
        this->m_Sht.getTemperatureHumidity(m);
        this->m_data.env.TempC = m.Temperature;
        this->m_data.env.Humidity = m.Humidity;
        this->m_data.flags |= Flags::TH;
        }

    if (this->m_data.co2ppm.CO2ppm != 0.0f)
        {
        this->m_data.flags |= Flags::CO2;
        }

    if (this->m_fIps7100)
        {
        m_Ips.updateData();

        this->m_data.particle.Count[0] = m_Ips.getPC01Data();
        this->m_data.particle.Count[1] = m_Ips.getPC03Data();
        this->m_data.particle.Count[2] = m_Ips.getPC05Data();
        this->m_data.particle.Count[3] = m_Ips.getPC10Data();
        this->m_data.particle.Count[4] = m_Ips.getPC25Data();
        this->m_data.particle.Count[5] = m_Ips.getPC50Data();
        this->m_data.particle.Count[6] = m_Ips.getPC100Data();

        this->m_data.particle.Mass[0] = m_Ips.getPM01Data();
        this->m_data.particle.Mass[1] = m_Ips.getPM03Data();
        this->m_data.particle.Mass[2] = m_Ips.getPM05Data();
        this->m_data.particle.Mass[3] = m_Ips.getPM10Data();
        this->m_data.particle.Mass[4] = m_Ips.getPM25Data();
        this->m_data.particle.Mass[5] = m_Ips.getPM50Data();
        this->m_data.particle.Mass[6] = m_Ips.getPM100Data();

        this->m_data.flags |= Flags::PM;
        }

    if (this->m_fAds131m04)
        {
        std::uint8_t channel0 = 0;
        std::uint8_t channel1 = 1;
        std::uint8_t channel2 = 2;
        std::uint8_t channel3 = 3;

        auto voltage = this->m_Ads.readVoltage(channel0);
        auto concentration = this->getCOConcentration(voltage);
        this->m_data.gases.CO = concentration;
        this->m_data.flags |= Flags::CO;

        voltage = m_Ads.readVoltage(channel1);
        concentration = this->getNO2Concentration(voltage);
        this->m_data.gases.NO2 = concentration;
        this->m_data.flags |= Flags::NO2;

        voltage = m_Ads.readVoltage(channel2);
        concentration = this->getO3Concentration(voltage);
        this->m_data.gases.O3 = concentration;
        this->m_data.flags |= Flags::O3;

        voltage = m_Ads.readVoltage(channel3);
        concentration = this->getSO2Concentration(voltage);
        this->m_data.gases.SO2 = concentration;
        this->m_data.flags |= Flags::SO2;
        }

    if (this->m_GpsSamM8q)
        {
        this->m_data.position.Latitude = m_Gps.getLatitude();
        this->m_data.position.Longitude = m_Gps.getLongitude();
        this->m_data.position.UnixTime = m_Gps.getUnixEpoch();
        this->m_data.flags |= Flags::GPS;
        }
    }

/****************************************************************************\
|
|   Start uplink of data
|
\****************************************************************************/

void cMeasurementLoop::startTransmission(
    cMeasurementLoop::TxBuffer_t &b
    )
    {
    auto const savedLed = gLed.Set(McciCatena::LedPattern::Off);
    gLed.Set(McciCatena::LedPattern::Sending);

    // by using a lambda, we can access the private contents
    auto sendBufferDoneCb =
        [](void *pClientData, bool fSuccess)
            {
            auto const pThis = (cMeasurementLoop *)pClientData;
            pThis->m_txpending = false;
            pThis->m_txcomplete = true;
            pThis->m_txerr = ! fSuccess;
            pThis->m_fsm.eval();
            };

    bool fConfirmed = false;
    if (gCatena.GetOperatingFlags() &
        static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fConfirmedUplink))
        {
        gCatena.SafePrintf("requesting confirmed tx\n");
        fConfirmed = true;
        }

    this->m_txpending = true;
    this->m_txcomplete = this->m_txerr = false;

    if (! gLoRaWAN.SendBuffer(b.getbase(), b.getn(), sendBufferDoneCb, (void *)this, fConfirmed))
        {
        // uplink wasn't launched.
        this->m_txcomplete = true;
        this->m_txerr = true;
        this->m_fsm.eval();
        }
    }

void cMeasurementLoop::sendBufferDone(bool fSuccess)
    {
    this->m_txpending = false;
    this->m_txcomplete = true;
    this->m_txerr = ! fSuccess;
    this->m_fsm.eval();
    }

/****************************************************************************\
|
|   The Polling function --
|
\****************************************************************************/

void cMeasurementLoop::poll()
    {
    bool fEvent;

    // no need to evaluate unless something happens.
    fEvent = false;

    // if we're not active, and no request, nothing to do.
    if (! this->m_active)
        {
        if (! this->m_rqActive)
            return;

        // we're asked to go active. We'll want to eval.
        fEvent = true;
        }

    auto const msToNext = this->m_Scd.getMsToNextMeasurement();
    if (msToNext < 20)
        updateScd30Measurements();

	   if (this->m_fTimerActive)
	        {
	        if ((millis() - this->m_timer_start) >= this->m_timer_delay)
	            {
	            this->m_fTimerActive = false;
	            this->m_fTimerEvent = true;
	            fEvent = true;
	            }
        }

    // check the transmit time.
    if (this->m_UplinkTimer.peekTicks() != 0)
        {
        fEvent = true;
        }

    if (fEvent)
        this->m_fsm.eval();

    this->m_data.Vbus = gCatena.ReadVbus();
    setVbus(this->m_data.Vbus);
    }

/****************************************************************************\
|
|   Update the TxCycle count.
|
\****************************************************************************/

void cMeasurementLoop::updateTxCycleTime()
    {
    auto txCycleCount = this->m_txCycleCount;

    // update the sleep parameters
    if (txCycleCount > 1)
            {
            // values greater than one are decremented and ultimately reset to default.
            this->m_txCycleCount = txCycleCount - 1;
            }
    else if (txCycleCount == 1)
            {
            // it's now one (otherwise we couldn't be here.)
            gCatena.SafePrintf("resetting tx cycle to default: %u\n", this->m_txCycleSec_Permanent);

            this->setTxCycleTime(this->m_txCycleSec_Permanent, 0);
            }
    else
            {
            // it's zero. Leave it alone.
            }
    }

/****************************************************************************\
|
|   Handle sleep between measurements
|
\****************************************************************************/

void cMeasurementLoop::sleep()
    {
    const bool fDeepSleep = checkDeepSleep();

    if (! this->m_fPrintedSleeping)
            this->doSleepAlert(fDeepSleep);

    if (fDeepSleep)
            this->doDeepSleep();
    }

// for now, we simply don't allow deep sleep. In the future might want to
// use interrupts on activity to wake us up; then go back to sleep when we've
// seen nothing for a while.
bool cMeasurementLoop::checkDeepSleep()
    {
    bool const fDeepSleepTest = gCatena.GetOperatingFlags() &
                    static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fDeepSleepTest);
    bool fDeepSleep;
    std::uint32_t const sleepInterval = this->m_UplinkTimer.getRemaining() / 1000;

    if (sleepInterval < 2)
        fDeepSleep = false;
    else if (fDeepSleepTest)
        {
        fDeepSleep = true;
        }
#ifdef USBCON
    else if (Serial.dtr())
        {
        fDeepSleep = false;
        }
#endif
    else if (gCatena.GetOperatingFlags() &
                static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fDisableDeepSleep))
        {
        fDeepSleep = false;
        }
    else if ((gCatena.GetOperatingFlags() &
                static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fUnattended)) != 0)
        {
        fDeepSleep = true;
        }
    else
        {
        fDeepSleep = false;
        }

    return fDeepSleep;
    }

void cMeasurementLoop::doSleepAlert(bool fDeepSleep)
    {
    this->m_fPrintedSleeping = true;

    if (fDeepSleep)
        {
        bool const fDeepSleepTest =
                gCatena.GetOperatingFlags() &
                    static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fDeepSleepTest);
        const uint32_t deepSleepDelay = fDeepSleepTest ? 10 : 30;

        gCatena.SafePrintf("using deep sleep in %u secs"
#ifdef USBCON
                            " (USB will disconnect while asleep)"
#endif
                            ": ",
                            deepSleepDelay
                            );

        // sleep and print
        gLed.Set(McciCatena::LedPattern::TwoShort);

        for (auto n = deepSleepDelay; n > 0; --n)
            {
            uint32_t tNow = millis();

            while (uint32_t(millis() - tNow) < 1000)
                {
                gCatena.poll();
                yield();
                }
            gCatena.SafePrintf(".");
            }
        gCatena.SafePrintf("\nStarting deep sleep.\n");
        uint32_t tNow = millis();
        while (uint32_t(millis() - tNow) < 100)
            {
            gCatena.poll();
            yield();
            }
        }
    else
        gCatena.SafePrintf("using light sleep\n");
    }

void cMeasurementLoop::doDeepSleep()
    {
    // bool const fDeepSleepTest = gCatena.GetOperatingFlags() &
    //                         static_cast<uint32_t>(gCatena.OPERATING_FLAGS::fDeepSleepTest);
    std::uint32_t const sleepInterval = this->m_UplinkTimer.getRemaining() / 1000;

    if (sleepInterval == 0)
        return;

    /* ok... now it's time for a deep sleep */
    gLed.Set(McciCatena::LedPattern::Off);
    this->deepSleepPrepare();

    /* sleep */
    gCatena.Sleep(sleepInterval);

    /* recover from sleep */
    this->deepSleepRecovery();

    /* and now... we're awake again. trigger another measurement */
    this->m_fsm.eval();
    }

void cMeasurementLoop::deepSleepPrepare(void)
    {
    if (this->m_fSleepScd30)
        {
        // stop the SCD30; we leave it running.
        this->m_Scd.end();
        }

    Serial.end();
    Wire.end();
    SPI.end();
    if (this->m_pSPI2 && this->m_fSpi2Active)
        {
        this->m_pSPI2->end();
        this->m_fSpi2Active = false;
        }
    pinMode(D11, INPUT);
    }

void cMeasurementLoop::deepSleepRecovery(void)
    {
    pinMode(D11, OUTPUT);
    digitalWrite(D11, HIGH);

    Serial.begin();
    Wire.begin();
    SPI.begin();
    //if (this->m_pSPI2)
    //    this->m_pSPI2->begin();

    if (this->m_fSleepScd30)
        {
        // start the SCD30, and make sure it passes the bring-up.
        // record success in m_fDiffPressure, which is used later
        // when collecting results to transmit.
        this->m_fScd30 = this->m_Scd.begin();
        this->m_fSleepScd30 = false;

        // if it didn't start, log a message.
        if (! this->m_fScd30)
            {
            if (gLog.isEnabled(gLog.DebugFlags::kError))
                gLog.printf(
                        gLog.kAlways,
                        "SCD30 begin() failed after sleep: status %s(%u)\n",
                        this->m_Scd.getLastErrorName(),
                        unsigned(this->m_Scd.getLastError())
                        );
            }
        }
    }

/****************************************************************************\
|
|  Time-out asynchronous measurements.
|
\****************************************************************************/

// set the timer
void cMeasurementLoop::setTimer(std::uint32_t ms)
    {
    this->m_timer_start = millis();
    this->m_timer_delay = ms;
    this->m_fTimerActive = true;
    this->m_fTimerEvent = false;
    }

void cMeasurementLoop::clearTimer()
    {
    this->m_fTimerActive = false;
    this->m_fTimerEvent = false;
    }

bool cMeasurementLoop::timedOut()
    {
    bool result = this->m_fTimerEvent;
    this->m_fTimerEvent = false;
    return result;
    }
