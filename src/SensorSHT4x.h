#pragma once
// #include "IncludeManager.h"
#ifdef SENSORMODULE
    #include "Sensor.h"

    // The SHT4x address is fixed by the part variant (SHT4x-Axxx = 0x44, -Bxxx = 0x45, -Cxxx = 0x46)
    #define SHT4X_ADDR 0x44

    // 8-bit commands
    #define SHT4X_MEAS_HIGHREP 0xFD
    #define SHT4X_MEAS_MEDREP 0xF6
    #define SHT4X_MEAS_LOWREP 0xE0
    #define SHT4X_READSERIAL 0x89
    #define SHT4X_SOFTRESET 0x94

    // Heater: OpenKNX hardware can supply up to 230mW (3.3V*70mA) on the UP1-SEN-8X.
    //         The 110 mW (0x2F, 0x24) and 200 mW (0x39, 0x32) commands could be added here.
    //         Playing it on the safe side and limiting to the 20 mW commands.
    //         But as with the SHT3x, the heater is never triggered by sensor steering...
    #define SHT4X_HEATER_20MW_1S 0x1E
    #define SHT4X_HEATER_20MW_100MS 0x15

    // max. measurement time is 8.3 ms (high repeatability), see datasheet table 5
    #define SHT4X_MEAS_WAIT_MS 10
    // Datasheet section 4.9: heater shall only be operated below 65 °C ambient
    #define SHT4X_HEATER_MAX_AMBIENT 65.0f
    // Time from heater start until the next heater start is allowed and measured values are published again
    #define SHT4X_HEATER_LOCKOUT_MS 30000

// non-blocking implementation of SHT4x Temp/Hum-Sensor
class SensorSHT4x : public Sensor
{
  protected:
    void reset();
    bool readSerial(uint32_t& eSerial);
    uint8_t crc8(const uint8_t* iData, uint8_t iLen);
    bool readFrame(uint8_t* eBuffer);
    bool getTempHum();
    bool writeCommand(uint8_t iCmd);

    float mTemp = NO_NUM;
    float mHumidity = NO_NUM;
    uint32_t mCommandSent = 0;   // time of last measurement command, 0 = no measurement pending
    bool mHeaterActive = false;  // heater pulse (incl. its measurement) in progress
    bool mHeaterUsed = false;    // heater lockout period in progress
    uint32_t mHeaterStart = 0;
    uint16_t mHeaterDuration = 0;

    uint8_t getSensorClass() override; // returns unique ID for this sensor type
    void sensorLoopInternal() override;
    float measureValue(MeasureType iMeasureType) override;

  public:
    SensorSHT4x(uint16_t iMeasureTypes, TwoWire* iWire);
    SensorSHT4x(uint16_t iMeasureTypes, TwoWire* iWire, uint8_t iAddress);
    virtual ~SensorSHT4x() {}

    bool begin() override;
    uint8_t getI2cSpeed() override;
    std::string logPrefix() override;

    // Starts a 20 mW heater pulse (1 s or 0.1 s). Non-blocking.
    // Returns false, if the request was refused (see implementation for the conditions).
    bool startHeater(bool iLongPulse);
};
#endif
