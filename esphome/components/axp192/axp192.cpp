#include "AXP192.h"
#include "esphome/core/log.h"

namespace esphome {
namespace axp192 {

static const char *TAG = "axp192.sensor";

AXP192Component::AXP192Component()
{
}

void AXP192Component::update()
{
  if (this->battery_voltage_sensor_ != nullptr)
  {
    float battery_voltage_v = GetBatVoltage();
    ESP_LOGD(TAG, "Got Battery Voltage=%f V", battery_voltage_v);
    this->battery_voltage_sensor_->publish_state(battery_voltage_v);
  }
  if (this->internal_temperature_sensor_ != nullptr)
  {
    float internal_temperature_c = GetTempInAXP192();
    ESP_LOGD(TAG, "Got Internal Temperature=%f C", internal_temperature_c);
    this->internal_temperature_sensor_->publish_state(internal_temperature_c);
  }
}
void AXP192Component::dump_config()
{
  ESP_LOGCONFIG(TAG, "AXP192:");
  LOG_I2C_DEVICE(this);

  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with AXP192 failed!");
    return;
  }
  LOG_UPDATE_INTERVAL(this);

  LOG_SENSOR("  ", "Battery Voltage", this->battery_voltage_sensor_);
  LOG_SENSOR("  ", "Internal Temperature", this->internal_temperature_sensor_);
}

float AXP192Component::get_setup_priority() const { return setup_priority::DATA; }


uint8_t AXP192Component::Read8bit(uint8_t Addr)
{
    uint8_t Data = 0;
    read_byte(Addr,&Data);
    return Data;
}

uint16_t AXP192Component::Read12Bit(uint8_t Addr)
{

    uint16_t Data = 0;
    uint8_t buf[2];
    this->read_bytes(Addr,buf,2);
    Data = ((buf[0] << 4) + buf[1]); //
    return Data;
}

uint16_t AXP192Component::Read13Bit(uint8_t Addr)
{
    uint16_t Data = 0;
    uint8_t buf[2];
    read_bytes(Addr,buf,2);
    Data = ((buf[0] << 5) + buf[1]); //
    return Data;
}

uint16_t AXP192Component::Read16bit( uint8_t Addr )
{
    uint32_t ReData = 0;
    uint8_t Buff[2];
    this->read_bytes(Addr, Buff, sizeof(Buff));
    for( int i = 0 ; i < sizeof(Buff) ; i++ )
    {
        ReData <<= 8;
        ReData |= Buff[i];
    }
    return ReData;
}

uint32_t AXP192Component::Read24bit( uint8_t Addr )
{
    uint32_t ReData = 0;
    uint8_t Buff[3];
    this->read_bytes(Addr, Buff, sizeof(Buff));
    for( int i = 0 ; i < sizeof(Buff) ; i++ )
    {
        ReData <<= 8;
        ReData |= Buff[i];
    }
    return ReData;
}

uint32_t AXP192Component::Read32bit( uint8_t Addr )
{
    uint32_t ReData = 0;
    uint8_t Buff[4];
    this->read_bytes(Addr, Buff, sizeof(Buff));
    for( int i = 0 ; i < sizeof(Buff) ; i++ )
    {
        ReData <<= 8;
        ReData |= Buff[i];
    }
    return ReData;
}

void AXP192Component::ScreenBreath(uint8_t brightness)
{
    if (brightness > 12)
    {
        brightness = 12;
    }
    uint8_t buf = Read8bit(0x28);
    write_byte(0x28, ((buf & 0x0f) | (brightness << 4)));
}

bool AXP192Component::GetBatState()
{
    if (Read8bit(0x01) | 0x20)
        return true;
    else
        return false;
}
//---------coulombcounter_from_here---------
//enable: void EnableCoulombcounter(void);
//disable: void DisableCOulombcounter(void);
//stop: void StopCoulombcounter(void);
//clear: void ClearCoulombcounter(void);
//get charge data: uint32_t GetCoulombchargeData(void);
//get discharge data: uint32_t GetCoulombdischargeData(void);
//get coulomb val affter calculation: float GetCoulombData(void);
//------------------------------------------
void AXP192Component::EnableCoulombcounter(void)
{
    write_byte(0xB8, 0x80);
}

void AXP192Component::DisableCoulombcounter(void)
{
    write_byte(0xB8, 0x00);
}

void AXP192Component::StopCoulombcounter(void)
{
    write_byte(0xB8, 0xC0);
}

void AXP192Component::ClearCoulombcounter(void)
{
    write_byte(0xB8, 0xA0);
}

uint32_t AXP192Component::GetCoulombchargeData(void)
{
    return Read32bit(0xB0);
}

uint32_t AXP192Component::GetCoulombdischargeData(void)
{
    return Read32bit(0xB4);
}

float AXP192Component::GetCoulombData(void)
{

    uint32_t coin = 0;
    uint32_t coout = 0;

    coin = GetCoulombchargeData();
    coout = GetCoulombdischargeData();

    //c = 65536 * current_LSB * (coin - coout) / 3600 / ADC rate
    //Adc rate can be read from 84H ,change this variable if you change the ADC reate
    float ccc = 65536 * 0.5 * (coin - coout) / 3600.0 / 25.0;
    return ccc;
}

// Cut all power, except for LDO1 (RTC)
void AXP192Component::PowerOff(void)
{
    write_byte(0x32, Read8bit(0x32) | 0b10000000);
}

void AXP192Component::SetAdcState(bool state)
{
    // Enable / Disable all ADCs
    write_byte(0x82, state ? 0xff : 0x00);
}

void AXP192Component::PrepareToSleep(void)
{
    // Disable ADCs
    SetAdcState(false);

    // Turn LED off
    SetLed(false);

    // Turn LCD backlight off
    SetDCDC3(false);
}

// Get current battery level
float AXP192Component::GetBatteryLevel(void)
{
    const float batVoltage = GetBatVoltage();
    const float batPercentage = 
        (batVoltage < 3.248088) 
        ? (0) 
        : (batVoltage - 3.120712) * 100;       
    return (batPercentage <= 100) ? batPercentage : 100;    
}

void AXP192Component::RestoreFromLightSleep(void)
{
    // Turn LCD backlight on
    SetDCDC3(true);

    // Turn LED on
    SetLed(true);

    // Enable ADCs
    SetAdcState(true);
}

uint8_t AXP192Component::GetWarningLeve(void)
{
    Wire1.beginTransmission(0x34);
    Wire1.write(0x47);
    Wire1.endTransmission();
    Wire1.requestFrom(0x34, 1);
    uint8_t buf = Wire1.read();
    return (buf & 0x01);
}

// -- sleep
void AXP192Component::DeepSleep(uint64_t time_in_us)
{
    PrepareToSleep();

    if (time_in_us > 0)
    {
        esp_sleep_enable_timer_wakeup(time_in_us);
    }
    else
    {
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
    }
    (time_in_us == 0) ? esp_deep_sleep_start() : esp_deep_sleep(time_in_us);

    // Never reached - after deep sleep ESP32 restarts
}

void AXP192Component::LightSleep(uint64_t time_in_us)
{
    PrepareToSleep();

    if (time_in_us > 0)
    {
        esp_sleep_enable_timer_wakeup(time_in_us);
    }
    else
    {
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
    }
    esp_light_sleep_start();

    RestoreFromLightSleep();
}

uint8_t AXP192Component::GetWarningLevel(void)
{
    return Read8bit(0x47) & 0x01;
}

float AXP192Component::GetBatVoltage()
{
    float ADCLSB = 1.1 / 1000.0;
    uint16_t ReData = Read12Bit(0x78);
    return ReData * ADCLSB;
}

float AXP192Component::GetBatCurrent()
{
    float ADCLSB = 0.5;
    uint16_t CurrentIn = Read13Bit(0x7A);
    uint16_t CurrentOut = Read13Bit(0x7C);
    return (CurrentIn - CurrentOut) * ADCLSB;
}

float AXP192Component::GetVinVoltage()
{
    float ADCLSB = 1.7 / 1000.0;
    uint16_t ReData = Read12Bit(0x56);
    return ReData * ADCLSB;
}

float AXP192Component::GetVinCurrent()
{
    float ADCLSB = 0.625;
    uint16_t ReData = Read12Bit(0x58);
    return ReData * ADCLSB;
}

float AXP192Component::GetVBusVoltage()
{
    float ADCLSB = 1.7 / 1000.0;
    uint16_t ReData = Read12Bit(0x5A);
    return ReData * ADCLSB;
}

float AXP192Component::GetVBusCurrent()
{
    float ADCLSB = 0.375;
    uint16_t ReData = Read12Bit(0x5C);
    return ReData * ADCLSB;
}

float AXP192Component::GetTempInAXP192()
{
    float ADCLSB = 0.1;
    const float OFFSET_DEG_C = -144.7;
    uint16_t ReData = Read12Bit(0x5E);
    return OFFSET_DEG_C + ReData * ADCLSB;
}

float AXP192Component::GetBatPower()
{
    float VoltageLSB = 1.1;
    float CurrentLCS = 0.5;
    uint32_t ReData = Read24bit(0x70);
    return VoltageLSB * CurrentLCS * ReData / 1000.0;
}

float AXP192Component::GetBatChargeCurrent()
{
    float ADCLSB = 0.5;
    uint16_t ReData = Read12Bit(0x7A);
    return ReData * ADCLSB;
}
float AXP192Component::GetAPSVoltage()
{
    float ADCLSB = 1.4 / 1000.0;
    uint16_t ReData = Read12Bit(0x7E);
    return ReData * ADCLSB;
}

float AXP192Component::GetBatCoulombInput()
{
    uint32_t ReData = Read32bit(0xB0);
    return ReData * 65536 * 0.5 / 3600 / 25.0;
}

float AXP192Component::GetBatCoulombOut()
{
    uint32_t ReData = Read32bit(0xB4);
    return ReData * 65536 * 0.5 / 3600 / 25.0;
}

void AXP192Component::SetCoulombClear()
{
    write_byte(0xB8, 0x20);
}

void AXP192Component::SetLDO2(bool State)
{
    uint8_t buf = Read8bit(0x12);
    if (State == true)
        buf = (1 << 2) | buf;
    else
        buf = ~(1 << 2) & buf;
    write_byte(0x12, buf);
}

void AXP192Component::SetDCDC3(bool State)
{
    uint8_t buf = Read8bit(0x12);
    if (State == true)
        buf = (1 << 1) | buf;
    else
        buf = ~(1 << 1) & buf;
    write_byte(0x12, buf);
}

uint8_t AXP192Component::AXPInState()
{
    return Read8bit(0x00);
}
bool AXP192Component::isACIN()
{
    return ( Read8bit(0x00) & 0x80 ) ? true : false;
}
bool AXP192Component::isCharging()
{
    return ( Read8bit(0x00) & 0x04 ) ? true : false;
}
bool AXP192Component::isVBUS()
{
    return ( Read8bit(0x00) & 0x20 ) ? true : false;
}

void AXP192Component::SetLDOVoltage(uint8_t number, uint16_t voltage)
{
    voltage = (voltage > 3300) ? 15 : (voltage / 100) - 18;
    switch (number)
    {
    //uint8_t reg, data;
    case 2:
        write_byte(0x28, (Read8bit(0x28) & 0X0F) | (voltage << 4));
        break;
    case 3:
        write_byte(0x28, (Read8bit(0x28) & 0XF0) | voltage);
        break;
    }
}

void AXP192Component::SetDCVoltage(uint8_t number, uint16_t voltage)
{
    uint8_t addr;
    if (number > 2)
        return;
    voltage = (voltage < 700) ? 0 : (voltage - 700) / 25;
    switch (number)
    {
    case 0:
        addr = 0x26;
        break;
    case 1:
        addr = 0x25;
        break;
    case 2:
        addr = 0x27;
        break;
    }
    write_byte(addr, (Read8bit(addr) & 0X80) | (voltage & 0X7F));
}

void AXP192Component::SetESPVoltage(uint16_t voltage)
{
    if (voltage >= 3000 && voltage <= 3400)
    {
        SetDCVoltage(0, voltage);
    }
}
void AXP192Component::SetLcdVoltage(uint16_t voltage)
{
    if (voltage >= 2500 && voltage <= 3300)
    {
        SetDCVoltage(2, voltage);
    }
}

void AXP192Component::SetLDOEnable(uint8_t number, bool state)
{
    uint8_t mark = 0x01;
    if ((number < 2) || (number > 3))
        return;

    mark <<= number;
    if (state)
    {
        write_byte(0x12, (Read8bit(0x12) | mark));
    }
    else
    {
        write_byte(0x12, (Read8bit(0x12) & (~mark)));
    }
}

void AXP192Component::SetLCDRSet(bool state)
{
    uint8_t reg_addr = 0x96;
    uint8_t gpio_bit = 0x02;
    uint8_t data;
    data = Read8bit(reg_addr);

    if (state)
    {
        data |= gpio_bit;
    }
    else
    {
        data &= ~gpio_bit;
    }

    write_byte(reg_addr, data);
}

void AXP192Component::SetBusPowerMode(uint8_t state)
{
    uint8_t data;
    if (state == 0)
    {
        data = Read8bit(0x91);
        write_byte(0x91, (data & 0X0F) | 0XF0);

        data = Read8bit(0x90);
        write_byte(0x90, (data & 0XF8) | 0X02); //set GPIO0 to LDO OUTPUT , pullup N_VBUSEN to disable supply from BUS_5V

        data = Read8bit(0x91);

        data = Read8bit(0x12);         //read reg 0x12
        write_byte(0x12, data | 0x40); //set EXTEN to enable 5v boost
    }
    else
    {
        data = Read8bit(0x12);         //read reg 0x10
        write_byte(0x12, data & 0XBF); //set EXTEN to disable 5v boost

        //delay(2000);

        data = Read8bit(0x90);
        write_byte(0x90, (data & 0xF8) | 0X01); //set GPIO0 to float , using enternal pulldown resistor to enable supply from BUS_5VS
    }
}

void AXP192Component::SetLed(uint8_t state)
{
    uint8_t reg_addr=0x94;
    uint8_t data;
    data=Read8bit(reg_addr);

    if(state)
    {
      data=data&0XFD;
    }
    else
    {
      data|=0X02;
    }

    write_byte(reg_addr,data);
}

//set led state(GPIO high active,set 1 to enable amplifier)
void AXP192Component::SetSpkEnable(uint8_t state)
{
    uint8_t reg_addr=0x94;
    uint8_t gpio_bit=0x04;
    uint8_t data;
    data=Read8bit(reg_addr);

    if(state)
    {
      data|=gpio_bit;
    }
    else
    {
      data&=~gpio_bit;
    }

    write_byte(reg_addr,data);
}

void AXP192Component::SetCHGCurrent(uint8_t state)
{
    uint8_t data = Read8bit(0x33);
    data &= 0xf0;
    data = data | ( state & 0x0f );
    write_byte(0x33,data);
}

void AXP192M5Core2::initialize() {
    //Specfic init for M5Core2
    ESP_LOGD(TAG, "Beginning..");
    //Wire1.begin(21, 22);
    //Wire1.setClock(400000);

    //AXP192 30H
    write_byte(0x30, (Read8bit(0x30) & 0x04) | 0X02);
    ESP_LOGD(TAG,"axp: vbus limit off\n");

    //AXP192 GPIO1:OD OUTPUT
    write_byte(0x92, Read8bit(0x92) & 0xf8);
    ESP_LOGD(TAG,"axp: gpio1 init\n");

    //AXP192 GPIO2:OD OUTPUT
    write_byte(0x93, Read8bit(0x93) & 0xf8);
    ESP_LOGD(TAG,"axp: gpio2 init\n");

    //AXP192 RTC CHG
    write_byte(0x35, (Read8bit(0x35) & 0x1c) | 0xa2);
    ESP_LOGD(TAG,"axp: rtc battery charging enabled\n");

    SetESPVoltage(3350);
    ESP_LOGD(TAG,"axp: esp32 power voltage was set to 3.35v\n");

    SetLcdVoltage(2800);
    ESP_LOGD(TAG,"axp: lcd backlight voltage was set to 2.80v\n");

    SetLDOVoltage(2, 3300); //Periph power voltage preset (LCD_logic, SD card)
    ESP_LOGD(TAG,"axp: lcd logic and sdcard voltage preset to 3.3v\n");

    SetLDOVoltage(3, 2000); //Vibrator power voltage preset
    ESP_LOGD(TAG,"axp: vibrator voltage preset to 2v\n");

    SetLDOEnable(2, true);
    SetDCDC3(true); // LCD backlight
    SetLed(true);

    SetCHGCurrent(kCHG_100mA);
    //SetAxpPriphPower(1);
    //ESP_LOGD(TAG,"axp: lcd_logic and sdcard power enabled\n\n");

    //pinMode(39, INPUT_PULLUP);

    //AXP192 GPIO4
    write_byte(0X95, (Read8bit(0x95) & 0x72) | 0X84);

    write_byte(0X36, 0X4C);

    write_byte(0x82,0xff);

    SetLCDRSet(0);
    delay(100);
    SetLCDRSet(1);
    delay(100);
    // I2C_WriteByteDataAt(0X15,0XFE,0XFF);

    //  bus power mode_output
    SetBusPowerMode(kMBusModeOutput);
}




}}