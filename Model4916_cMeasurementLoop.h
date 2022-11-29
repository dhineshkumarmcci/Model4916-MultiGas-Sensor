/*

Module: Model4916_cMeasurementLoop.h

Function:
    cMeasurementLoop definitions.

Copyright:
    See accompanying LICENSE file for copyright and license information.

Author:
    Dhinesh Kumar Pitchai, MCCI Corporation   November 2022

*/

#ifndef _Model4916_cMeasurementLoop_h_
# define _Model4916_cMeasurementLoop_h_

#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Catena_FSM.h>
#include <Catena_Led.h>
#include <Catena_Log.h>
#include <Catena_Mx25v8035f.h>
#include <Catena_PollableInterface.h>
#include <Catena_Timer.h>
#include <Catena_BootloaderApi.h>
#include <Catena_Download.h>
#include <Catena_TxBuffer.h>
#include <Catena.h>
#include <mcciadk_baselib.h>
#include <stdlib.h>
#include <bsec.h>
#include <Catena-SHT3x.h>
#include <MCCI_Catena_SCD30.h>
#include <MCCI_Catena_IPS-7100.h>
#include <MCCI_Catena_ADS131M04.h>
#include <MCCI_Catena_SAM-M8Q.h>

#include <cstdint>

extern McciCatena::Catena gCatena;
extern McciCatena::Catena::LoRaWAN gLoRaWAN;
extern McciCatena::StatusLed gLed;

using namespace McciCatenaSht3x;
using namespace McciCatenaAds131m04;
using namespace McciCatenaIps7100;

namespace McciModel4916 {

/****************************************************************************\
|
|   An object to represent the uplink activity
|
\****************************************************************************/

class cMeasurementBase
    {

    };

class cMeasurementFormat : public cMeasurementBase
    {
public:
    // buffer size for uplink data
    static constexpr size_t kTxBufferSize = 46;

    // message format
    static constexpr uint8_t kMessageFormat = 0x27;

    enum class Flags : uint16_t
            {
            Vbat = 1 << 0,      // vBat
            Boot = 1 << 1,      // boot count
            TH = 1 << 2,        // temperature, humidity
            GPS = 1 << 3,       // latitude, longitude
            PM = 1 << 4,        // Particle
            CO2 = 1 << 5,       // carbondioxide
            CO = 1 << 6,        // carbon-monoxide
            NO2 = 1 << 7,       // nitrogen-dioxide
            O3 = 1 << 8,        // ozone gas
            SO2 = 1 << 9,       // sulfur-dioxide
            TVOC = 1 << 10,     // TVOC
            IAQ = 1 << 11,      // Air Quality Index
            };

    // the structure of a measurement
    struct Measurement
        {
        //----------------
        // the subtypes:
        //----------------

        // compost temperature with SHT
        struct Env
            {
            // compost temperature (in degrees C)
            float                   TempC;
            // compost humidity (in percentage)
            float                   Humidity;
            };

        // measure co2ppm
        struct CO2ppm
            {
            float                   CO2ppm;
            };

        // measure particle with IPS-7100
        struct Particle
            {
            float                   Mass[6];
            std::uint32_t           Count[6];
            };

        // measures spec sensor data
        struct Gases
            {
            float                   CO;
            float                   NO2;
            float                   O3;
            float                   SO2;
            };

        // measure particle with IPS-7100
        struct Position
            {
            float                   Latitude;
            float                   Longitude;
            uint32_t                UnixTime;
            };

        //---------------------------
        // the actual members as POD
        //---------------------------

        // flags of entries that are valid.
        Flags                   	flags;
        // measured battery voltage, in volts
        float                       Vbat;
        // measured system Vdd voltage, in volts
        float                       Vsystem;
        // measured USB bus voltage, in volts.
        float                       Vbus;
        // boot count
        uint32_t                    BootCount;
        // compost temperature at bottom
        Env                         env;
        // measure co2ppm
        CO2ppm                      co2ppm;
        // measure particle
        Particle                    particle;
        // measure different gases
        Gases                       gases;
        // get position and time
        Position                    position;
        };
    };

class cMeasurementLoop : public McciCatena::cPollableObject
    {
public:
	// some parameters
	using MeasurementFormat = McciModel4916::cMeasurementFormat;
	using Measurement = MeasurementFormat::Measurement;
	using Flags = MeasurementFormat::Flags;
	static constexpr std::uint8_t kMessageFormat = MeasurementFormat::kMessageFormat;
    static constexpr std::uint8_t kSdCardCSpin = D11;

    enum OPERATING_FLAGS : uint32_t
        {
        fUnattended = 1 << 0,
        fManufacturingTest = 1 << 1,
        fConfirmedUplink = 1 << 16,
        fDisableDeepSleep = 1 << 17,
        fQuickLightSleep = 1 << 18,
        fDeepSleepTest = 1 << 19,
        };

    enum DebugFlags : std::uint32_t
        {
        kError      = 1 << 0,
        kWarning    = 1 << 1,
        kTrace      = 1 << 2,
        kInfo       = 1 << 3,
        };

    // constructor
    cMeasurementLoop(
            McciCatenaSht3x::cSHT3x& sht3x,
            McciCatenaScd30::cSCD30& scd30,
            McciCatenaIps7100::cIPS7100& ips7100
            )
        : m_Sht(sht3x)
        , m_Scd(scd30)
        , m_Ips(ips7100)
        , m_txCycleSec_Permanent(6 * 60) // default uplink interval
        , m_txCycleSec(30)                    // initial uplink interval
        , m_txCycleCount(10)                   // initial count of fast uplinks
        , m_DebugFlags(DebugFlags(kError | kTrace))
        {};

    // neither copyable nor movable
    cMeasurementLoop(const cMeasurementLoop&) = delete;
    cMeasurementLoop& operator=(const cMeasurementLoop&) = delete;
    cMeasurementLoop(const cMeasurementLoop&&) = delete;
    cMeasurementLoop& operator=(const cMeasurementLoop&&) = delete;

    enum class State : std::uint8_t
        {
        stNoChange = 0, // this name must be present: indicates "no change of state"
        stInitial,      // this name must be present: it's the starting state.
        stInactive,     // parked; not doing anything.
        stSleeping,     // active; sleeping between measurements
        stWarmup,       // transition from inactive to measure, get some data.
        stMeasure,      // take measurents
        stTransmit,     // transmit data
        stFinal,        // this name must be present, it's the terminal state.
        };

    static constexpr const char *getStateName(State s)
        {
        switch (s)
            {
            case State::stNoChange: return "stNoChange";
            case State::stInitial:  return "stInitial";
            case State::stInactive: return "stInactive";
            case State::stSleeping: return "stSleeping";
            case State::stWarmup:   return "stWarmup";
            case State::stMeasure:  return "stMeasure";
            case State::stTransmit: return "stTransmit";
            case State::stFinal:    return "stFinal";
            default:                return "<<unknown>>";
            }
        }

    // concrete type for uplink data buffer
    using TxBuffer_t = McciCatena::AbstractTxBuffer_t<MeasurementFormat::kTxBufferSize>;
    using TxBufferBase_t = McciCatena::AbstractTxBufferBase_t;

    // initialize measurement FSM.
    void begin();
    void end();
    void setTxCycleTime(
        std::uint32_t txCycleSec,
        std::uint32_t txCycleCount
        )
        {
        this->m_txCycleSec = txCycleSec;
        this->m_txCycleCount = txCycleCount;

        this->m_UplinkTimer.setInterval(txCycleSec * 1000);
        if (this->m_UplinkTimer.peekTicks() != 0)
            this->m_fsm.eval();
        }
    std::uint32_t getTxCycleTime()
        {
        return this->m_txCycleSec;
        }
    virtual void poll() override;

    void setBme680(bool fEnable)
        {
        this->m_fBme680 = fEnable;
        }

    void setVbus(float Vbus)
        {
        // set threshold value as 4.0V as there is reverse voltage
        // in vbus(~3.5V) while powered from battery in 4916.
        this->m_fUsbPower = (Vbus > 4.0f) ? true : false;
        }

    void printSCDinfo()
        {
        auto const info = m_Scd.getInfo();
        gCatena.SafePrintf(
                    "Found sensor: firmware version %u.%u\n",
                    info.FirmwareVersion / 256u,
                    info.FirmwareVersion & 0xFFu
                    );
        gCatena.SafePrintf("  Automatic Sensor Calibration: %u\n", info.fASC_status);
        gCatena.SafePrintf("  Sample interval:      %6u secs\n", info.MeasurementInterval);
        gCatena.SafePrintf("  Forced Recalibration: %6u ppm\n", info.ForcedRecalibrationValue);
        gCatena.SafePrintf("  Temperature Offset:   %6d centi-C\n", info.TemperatureOffset);
        gCatena.SafePrintf("  Altitude:             %6d meters\n", info.AltitudeCompensation);
        }

    void configGps()
        {
        // Set Configuration
        m_Gps.configureMessage(UBX_CLASS_NAV, UBX_NAV_PVT, COM_PORT_I2C, 1);

        // Enable GPS/Galileo/Glonass and disable others
        m_Gps.enableGNSS(true, SAM_M8Q_ID_GPS);
        m_Gps.enableGNSS(true, SAM_M8Q_ID_GALILEO);
        m_Gps.enableGNSS(true, SAM_M8Q_ID_GLONASS);
        m_Gps.enableGNSS(false, SAM_M8Q_ID_SBAS);
        m_Gps.enableGNSS(false, SAM_M8Q_ID_BEIDOU);
        m_Gps.enableGNSS(false, SAM_M8Q_ID_IMES);
        m_Gps.enableGNSS(false, SAM_M8Q_ID_QZSS);

        delay (2000);

        if (m_Gps.isGNSSenabled(SAM_M8Q_ID_GPS) == true && \
            m_Gps.isGNSSenabled(SAM_M8Q_ID_GALILEO) == true && \
            m_Gps.isGNSSenabled(SAM_M8Q_ID_GLONASS) == true
            )
            {
            gCatena.SafePrintf("GNSS Module enabled successfully\n");
            }
        else
            {
            gCatena.SafePrintf("GNSS Module is not enabled\n");
            }

        // Set the I2C port to output
        m_Gps.setI2COutput(COM_TYPE_UBX);

        // Save Configuration
        m_Gps.saveConfiguration();
        }

    float getCOConcentration(float voltage)
        {
        float Vgas;
        float gasConcentration;

        Vgas = voltage - this->m_vGasZero;
        gasConcentration = this->m_calibrationFactorCO * Vgas;

        return gasConcentration;
        }

    float getNO2Concentration(float voltage)
        {
        float Vgas;
        float gasConcentration;

        Vgas = voltage - this->m_vGasZero;
        gasConcentration = this->m_calibrationFactorNO2 * Vgas;

        return gasConcentration;
        }

    float getO3Concentration(float voltage)
        {
        float Vgas;
        float gasConcentration;

        Vgas = voltage - this->m_vGasZero;
        gasConcentration = this->m_calibrationFactorO3 * Vgas;

        return gasConcentration;
        }

    float getSO2Concentration(float voltage)
        {
        float Vgas;
        float gasConcentration;

        Vgas = voltage - this->m_vGasZero;
        gasConcentration = this->m_calibrationFactorSO2 * Vgas;

        return gasConcentration;
        }

    uint32_t getDecimal(float data)
        {
        uint32_t data100 = std::uint32_t (data * 100 + 0.5);
        uint32_t dataInt = data100/100;
        uint32_t dataFrac = data100 - (dataInt * 100);

        return dataFrac;
        }

    // request that the measurement loop be active/inactive
    void requestActive(bool fEnable);

    // return true if a given debug mask is enabled.
    bool isTraceEnabled(DebugFlags mask) const
        {
        return this->m_DebugFlags & mask;
        }

    // register an additional SPI for sleep/resume
    // can be called before begin().
    void registerSecondSpi(SPIClass *pSpi)
        {
        this->m_pSPI2 = pSpi;
        }

    /// bring up the SD card, if possible.
    bool checkSdCard();
    /// tear down the SD card.
    void sdFinish();
private:
    // sleep handling
    void sleep();
    bool checkDeepSleep();
    void doSleepAlert(bool fDeepSleep);
    void doDeepSleep();
    void deepSleepPrepare();
    void deepSleepRecovery();

    // read data
    void updateScd30Measurements();
    void updateSynchronousMeasurements();
    void resetMeasurements();

    // telemetry handling.
    void fillTxBuffer(TxBuffer_t &b, Measurement const & mData);
    void startTransmission(TxBuffer_t &b);
    void sendBufferDone(bool fSuccess);

    bool txComplete()
        {
        return this->m_txcomplete;
        }
    void updateTxCycleTime();

    // SD card handling
    bool initSdCard();

    bool writeSdCard(TxBuffer_t &b, Measurement const &mData);
    bool handleSdFirmwareUpdate();
    bool handleSdFirmwareUpdateCardUp();
    bool updateFromSd(const char *sFile, McciCatena::cDownload::DownloadRq_t rq);
    void sdPowerUp(bool fOn);
    void sdPrep();

    // timeout handling

    // set the timer
    void setTimer(std::uint32_t ms);
    // clear the timer
    void clearTimer();
    // test (and clear) the timed-out flag.
    bool timedOut();

    // instance data
private:
    McciCatena::cFSM<cMeasurementLoop, State> m_fsm;
    // evaluate the control FSM.
    State fsmDispatch(State currentState, bool fEntry);

    // second SPI class
    SPIClass                        *m_pSPI2;

    // BME680 Environmental sensor
    Bsec                            m_bme680;

    // SHT3x Environmental sensor
    McciCatenaSht3x::cSHT3x&        m_Sht;

    // SCD30 - CO2 sensor
    McciCatenaScd30::cSCD30&        m_Scd;
    char                            ts;
    int32_t                         t100;
    int32_t                         tint;
    int32_t                         tfrac;
    int32_t                         rh100;
    int32_t                         rhint;
    int32_t                         rhfrac;
    int32_t                         co2_100;
    int32_t                         co2int;
    int32_t                         co2frac;

    // IPS7100 - Particle sensor
    McciCatenaIps7100::cIPS7100&    m_Ips;

    // ADS131M04 - ADC for different spec sensor
    McciCatenaAds131m04::cADS131M04 m_Ads;
    float                           m_vGasZero = 1.65;
    float                           m_calibrationFactorCO = (1 / 0.000427);
    float                           m_calibrationFactorNO2 = (1 / -0.01535423);
    float                           m_calibrationFactorO3 = (1 / -0.01497998);
    float                           m_calibrationFactorSO2 = (1 / 0.00286);

    // SAM-M8Q - GPS for position and time
    SAM_M8Q                         m_Gps;

    // debug flags
    DebugFlags                      m_DebugFlags;

    // true if object is registered for polling.
    bool                            m_registered : 1;
    // true if object is running.
    bool                            m_running : 1;
    // true to request exit
    bool                            m_exit : 1;
    // true if in active uplink mode, false otehrwise.
    bool                            m_active : 1;

    // set true to request transition to active uplink mode; cleared by FSM
    bool                            m_rqActive : 1;
    // set true to request transition to inactive uplink mode; cleared by FSM
    bool                            m_rqInactive : 1;
    // set true if measurement is valid
    bool                            m_measurement_valid: 1;

    // set true if event timer times out
    bool                            m_fTimerEvent : 1;
    // set true while evenet timer is active.
    bool                            m_fTimerActive : 1;
    // set true if USB power is present.
    bool                            m_fUsbPower : 1;

    // set true while a transmit is pending.
    bool                            m_txpending : 1;
    // set true when a transmit completes.
    bool                            m_txcomplete : 1;
    // set true when a transmit complete with an error.
    bool                            m_txerr : 1;
    // set true when we've printed how we plan to sleep
    bool                            m_fPrintedSleeping : 1;
    // set true when SPI2 is active
    bool                            m_fSpi2Active: 1;
    // set true when we've BIN file in SD card to update
    bool                            m_fFwUpdate : 1;

    // set true if BME680 is present.
    bool                            m_fBme680 : 1;
    // set true if SHT3x is present
    bool                            m_fSht3x : 1;
    // set true if CO2 (SCD) is present
    bool                            m_fScd30 : 1;
    // set true if device enters Sleep state
    bool                            m_fSleepScd30 : 1;
    // set true if IPS-7100 is present
    bool                            m_fIps7100 : 1;
    // set true if ADS131M04 is present
    bool                            m_fAds131m04 : 1;
    // set true if SAM-M8q is present
    bool                            m_GpsSamM8q : 1;

    // uplink time control
    McciCatena::cTimer              m_UplinkTimer;
    std::uint32_t                   m_txCycleSec;
    std::uint32_t                   m_txCycleCount;
    std::uint32_t                   m_txCycleSec_Permanent;

    // simple timer for timing-out sensors.
    std::uint32_t                   m_timer_start;
    std::uint32_t                   m_timer_delay;

    // the current measurement
    Measurement                     m_data;

    TxBuffer_t                      m_FileTxBuffer;
    };

//
// operator overloads for ORing structured flags
//
static constexpr cMeasurementLoop::Flags operator| (const cMeasurementLoop::Flags lhs, const cMeasurementLoop::Flags rhs)
        {
        return cMeasurementLoop::Flags(uint8_t(lhs) | uint8_t(rhs));
        };

static constexpr cMeasurementLoop::Flags operator& (const cMeasurementLoop::Flags lhs, const cMeasurementLoop::Flags rhs)
        {
        return cMeasurementLoop::Flags(uint8_t(lhs) & uint8_t(rhs));
        };

static cMeasurementLoop::Flags operator|= (cMeasurementLoop::Flags &lhs, const cMeasurementLoop::Flags &rhs)
        {
        lhs = lhs | rhs;
        return lhs;
        };

} // namespace McciModel4916

#endif /* _Model4916_cMeasurementLoop_h_ */
