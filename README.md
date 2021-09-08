# Arduino MPM54304
Arduino library for [Monolithic Power Systems MPM54304](https://www.monolithicpower.com/en/mpm54304.html) quad channel buck converter.

Currently a work in process. Should mostly function but expect bugs.

## API

General:

```cpp
MPM54304(uint8_t addr = MPM54304_DEFAULT_ADDRESS);
~MPM54304();

bool begin();
bool update(bool forceWrite = false);

MPM54304_BUCK_CONFIG* getBuckConfigUnsafe(uint8_t buck);
MPM54304_SYSTEM_CONFIG* getSystemConfigUnsafe();

void setClock100KHz();
void setClock400KHz();
void setClock1000KHz();
void setClock3400KHz();

void printConfig();
```

System config:

```cpp
void enableBuck(uint8_t buck);
void disableBuck(uint8_t buck);
bool getBuckEnabled(uint8_t buck);
void setBuckEnabled(uint8_t buck, bool enable);

MPM54304UnderVoltageThreshold getUnderVoltageThreshold();
void setUnderVoltageThreshold(MPM54304UnderVoltageThreshold threshold);

MPM54304OutputPortSyncMode getOutputPortSyncMode();
void setOutputPortSyncMode(MPM54304OutputPortSyncMode syncMode);

MPM54304SwitchingFrequency getSwitchingFrequency();
void setSwitchingFrequency(MPM54304SwitchingFrequency freq);

bool getShutdownDelay();
void setShutdownDelay(bool enable);

uint8_t getCurrentI2CAddress();
bool isI2CAddressChangePending();
uint8_t getConfiguredI2CAddress();
bool setI2CAddress(uint8_t addr);

MPM54304OutputPinFunction getOutputPinFunction();
void setOutputPinFunction(MPM54304OutputPinFunction function);

MPM54304PowerGoodDelay getPowerGoodDelay();
void setPowerGoodDelay(MPM54304PowerGoodDelay pgDelay);

bool getParallelBuck12Enable();
bool getParallelBuck34Enable();
void enableParallelBuck12();
void enableParallelBuck34();
void disableParallelBuck12();
void disableParallelBuck34();
void setParallelBuck12Enable(bool enable);
void setParallelBuck34Enable(bool enable);

uint8_t getMTPConfigureCode();
void setMTPConfigureCode(uint8_t code);

uint8_t getMTPRevisionNumber();
void setMTPRevisionNumber(uint8_t revision);

bool getPowerGood(uint8_t buck);

bool getOverTemperatureWarning();

bool getOverTemperatureProtection();

uint8_t getVendorID();

bool getChecksumFlag();

uint8_t getCurrentMTPPageIndex();
```

Buck config:

```cpp
uint8_t getSoftStartDelayMillis(uint8_t buck);
void setSoftStartDelayMillis(uint8_t buck, uint8_t startDelay);

bool getAdditionalPhaseDelay(uint8_t buck);
void setAdditionalPhaseDelay(uint8_t buck, bool phaseDelay);

MPM54304SlewRate getSoftStartTime(uint8_t buck);
void setSoftStartTime(uint8_t buck, MPM54304SlewRate slewRate);

bool getVoltageOutLimitEnable(uint8_t buck);
void setVoltageOutLimitEnable(uint8_t buck, bool limitEnable);

MPM54304SwitchingMode getSwitchingMode(uint8_t buck);
void setSwitchingMode(uint8_t buck, MPM54304SwitchingMode switchingMode);

MPM54304CurrentLimit getCurrentLimit(uint8_t buck);
void setCurrentLimit(uint8_t buck, MPM54304CurrentLimit currentLimit);

bool getOverVoltageProtectionEnable(uint8_t buck);
void setOverVoltageProtectionEnable(uint8_t buck, bool enable);

MPM54304PhaseDelay getPhaseDelay(uint8_t buck);
void setPhaseDelay(uint8_t buck, MPM54304PhaseDelay phaseDelay);

bool getVoltageOutDischargeEnable(uint8_t buck);
void setVoltageOutDischargeEnable(uint8_t buck, bool enable);

MPM54304FeedbackRatio getFeedbackRatio(uint8_t buck);
void setFeedbackRatio(uint8_t buck, MPM54304FeedbackRatio feedbackRatio);

uint16_t getReferenceVoltage(uint8_t buck);
// sets the reference voltage. valid range is 550-1820mV, in steps of 10mV.
bool setReferenceVoltage(uint8_t buck, uint16_t millivolts);

// sets the nearest available output voltage (rounding up) by a combination of reference voltage and feedback ratio. returns the resulting reference voltage in mV, or 0 if the value was out of range.
uint16_t setOutputVoltage(uint8_t buck, float volts);
// gets the output voltage that will result from the current settings.
float getOutputVoltage(uint8_t buck);

// reads the system configuration block
bool readSystemConfig();
// writes the system configuration block
bool writeSystemConfig();

// reads the configuration block for one buck converter
bool readBuckConfig(uint8_t buck);
// writes the configuration block for one buck converter
bool writeBuckConfig(uint8_t buck);

// writes all configuration registers (system + all four bucks) at once, instead of separately.
bool writeAllConfig();
```

