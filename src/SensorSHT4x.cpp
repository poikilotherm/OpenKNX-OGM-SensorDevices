// #include "IncludeManager.h"
#ifdef SENSORMODULE
    #include "SensorSHT4x.h"
    #include <Wire.h>

// 10% duty cycle: one heater start per 10 s at most (longest pulse is 1 s)
static_assert(SHT4X_HEATER_LOCKOUT_MS >= 10 * 1000, "Heater lockout must keep the 10% duty cycle");

SensorSHT4x::SensorSHT4x(uint16_t iMeasureTypes, TwoWire* iWire)
    : Sensor(iMeasureTypes, iWire, SHT4X_ADDR){};

SensorSHT4x::SensorSHT4x(uint16_t iMeasureTypes, TwoWire* iWire, uint8_t iAddress)
    : Sensor(iMeasureTypes, iWire, iAddress){};

uint8_t SensorSHT4x::getSensorClass()
{
    return SENS_SHT4X;
}

std::string SensorSHT4x::logPrefix()
{
    return "Sensor<SHT4x>";
}

void SensorSHT4x::sensorLoopInternal()
{
    switch (pSensorState)
    {
        case Wakeup:
            Sensor::sensorLoopInternal();
            break;
        case Calibrate:
            writeCommand(SHT4X_MEAS_HIGHREP);
            Sensor::sensorLoopInternal();
            break;
        case Finalize:
            if (delayCheck(pSensorStateDelay, 100))
            {
                // fetch first measurement
                getTempHum();
                mCommandSent = 0;
                pSensorState = Running;
                pSensorStateDelay = millis();
            }
            break;
        case Running:
            if (mHeaterActive)
            {
                // the sensor is busy with the heater pulse and the measurement at its end,
                // it would NACK everything else
                if (delayCheck(mHeaterStart, mHeaterDuration + SHT4X_MEAS_WAIT_MS))
                {
                    // read to clear the sensor, the values are not published (see mHeaterUsed)
                    getTempHum();
                    mHeaterActive = false;
                    pSensorStateDelay = millis();
                }
                break;
            }
            if (mHeaterUsed && delayCheck(mHeaterStart, SHT4X_HEATER_LOCKOUT_MS))
                mHeaterUsed = false;

            // toggle between requesting a measurement and fetching it
            if (mCommandSent && delayCheck(mCommandSent, SHT4X_MEAS_WAIT_MS))
            {
                getTempHum();
                mCommandSent = 0;
            }
            else if (!mCommandSent && delayCheck(pSensorStateDelay, 2000))
            {
                if (writeCommand(SHT4X_MEAS_HIGHREP))
                    mCommandSent = millis();
                pSensorStateDelay = millis();
            }
            break;
        default:
            pSensorStateDelay = millis();
            break;
    }
}

float SensorSHT4x::measureValue(MeasureType iMeasureType)
{
    // sensor specifications are not valid while and shortly after heating
    if (mHeaterUsed)
        return NO_NUM;
    switch (iMeasureType)
    {
        case Temperature:
            return mTemp;
        case Humidity:
            return mHumidity;
        default:
            break;
    }
    return NO_NUM;
}

bool SensorSHT4x::begin()
{
    bool lResult = false;
    logInfoP("Starting sensor SHT4x... ");
    lResult = Sensor::begin(); // I2C ACK check
    if (lResult)
    {
        // a reset aborts any pending action (e.g. heater, measurement)
        // in case the sensor stayed powered while the MCU restarted
        mCommandSent = 0;
        mHeaterActive = false;
        mHeaterUsed = false;
        reset();
        // The serial number can only be read from a real SHT4x. It is read twice and must match:
        // pending data of an SHT3x at the same address can be read only once and would look
        // like a valid serial number the first time.
        uint32_t lSerial = 0;
        uint32_t lSerial2 = 0;
        lResult = readSerial(lSerial) && readSerial(lSerial2) && (lSerial == lSerial2);
        if (lResult)
            logDebugP("Serial number: 0x%08X", lSerial);
        else
            logErrorP("No SHT4x found at address 0x%02X (SHT3x connected?). Sensor is disabled!", pI2CAddress);
    }
    logResult(lResult);
    return lResult;
}

uint8_t SensorSHT4x::getI2cSpeed()
{
    return 4; // n * 100kHz
}

void SensorSHT4x::reset()
{
    writeCommand(SHT4X_SOFTRESET);
    delay(2); // soft reset time is max. 1 ms
}

bool SensorSHT4x::readSerial(uint32_t& eSerial)
{
    uint8_t lBuffer[6];
    if (!writeCommand(SHT4X_READSERIAL))
        return false;
    delay(1);
    if (!readFrame(lBuffer))
        return false;
    eSerial = ((uint32_t)lBuffer[0] << 24) | ((uint32_t)lBuffer[1] << 16) |
              ((uint32_t)lBuffer[3] << 8) | lBuffer[4];
    return true;
}

bool SensorSHT4x::startHeater(bool iLongPulse)
{
    if (pSensorState != Running || mHeaterActive || mHeaterUsed || mCommandSent)
    {
        logDebugP("Heater request refused: sensor is busy or in heater lockout");
        return false;
    }
    // unknown temperature is treated as unsafe
    if (mTemp <= (NO_NUM + 1.0f) || mTemp > SHT4X_HEATER_MAX_AMBIENT)
    {
        logDebugP("Heater request refused: temperature unknown or above %.0f °C", SHT4X_HEATER_MAX_AMBIENT);
        return false;
    }
    if (!writeCommand(iLongPulse ? SHT4X_HEATER_20MW_1S : SHT4X_HEATER_20MW_100MS))
        return false;
    mHeaterDuration = iLongPulse ? 1000 : 100;
    mHeaterStart = millis();
    mHeaterActive = true;
    mHeaterUsed = true;
    return true;
}

uint8_t SensorSHT4x::crc8(const uint8_t* iData, uint8_t iLen)
{
    const uint8_t cPolynomial(0x31);
    uint8_t lCrc(0xFF);

    for (uint8_t j = iLen; j; --j)
    {
        lCrc ^= *iData++;
        for (uint8_t i = 8; i; --i)
            lCrc = (lCrc & 0x80) ? (lCrc << 1) ^ cPolynomial : (lCrc << 1);
    }
    return lCrc;
}

// reads 2 words with CRC each (6 bytes) and checks the CRCs
// no data (NACK) is returned, if the sensor is still busy
bool SensorSHT4x::readFrame(uint8_t* eBuffer)
{
    pWire->requestFrom(pI2CAddress, 6);
    if (pWire->available() != 6)
        return false;
    for (uint8_t i = 0; i < 6; i++)
        eBuffer[i] = pWire->read();
    return eBuffer[2] == crc8(eBuffer, 2) && eBuffer[5] == crc8(eBuffer + 3, 2);
}

bool SensorSHT4x::getTempHum()
{
    uint8_t lBuffer[6];
    if (!readFrame(lBuffer))
        return false;

    int32_t lTemp = (int32_t)(((uint32_t)lBuffer[0] << 8) | lBuffer[1]);
    // simplified (65536 instead of 65535) integer version of:
    // temp = -45 + 175 * ticks / 65535, here in 1/100 °C
    lTemp = ((4375 * lTemp) >> 14) - 4500;
    mTemp = (float)lTemp / 100.0f;

    int32_t lHum = (int32_t)(((uint32_t)lBuffer[3] << 8) | lBuffer[4]);
    // simplified (65536 instead of 65535) integer version of:
    // humidity = -6 + 125 * ticks / 65535, here in 1/100 %RH
    lHum = ((3125 * lHum) >> 14) - 600;
    // crop to the physical range (datasheet 4.6)
    if (lHum < 0)
        lHum = 0;
    else if (lHum > 10000)
        lHum = 10000;
    mHumidity = (float)lHum / 100.0f;

    return true;
}

bool SensorSHT4x::writeCommand(uint8_t iCmd)
{
    pWire->beginTransmission(pI2CAddress);
    pWire->write(iCmd);
    return pWire->endTransmission() == 0;
}
#endif
