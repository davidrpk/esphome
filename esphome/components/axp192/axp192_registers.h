#pragma once

namespace esphome {
namespace axp192 {

// Group 1, power control

static const uint8_t AXP192_PW_STAT = 0x00;
static const uint8_t AXP192_CHR_STAT = 0x01;
static const uint8_t AXP192_OTG_VBUS_STAT = 0x04;
static const uint8_t AXP192_DATA_BUFFER0 = 0x06;
static const uint8_t AXP192_DATA_BUFFER1 = 0x07;
static const uint8_t AXP192_DATA_BUFFER2 = 0x08;
static const uint8_t AXP192_DATA_BUFFER3 = 0x09;
static const uint8_t AXP192_DATA_BUFFER4 = 0x0a;
static const uint8_t AXP192_DATA_BUFFER5 = 0x0b;

static const uint8_t AXP192_EXTEN_DCDC2_CONTROL = 0x10;
static const uint8_t AXP192_DCDC13_LDO23_CONTROL = 0x12;

static const uint8_t AXP192_DCDC2_VOLTAGE = 0x23;
static const uint8_t AXP192_DCDC2_SLOPE = 0x25;
static const uint8_t AXP192_DCDC1_VOLTAGE = 0x26;
static const uint8_t AXP192_DCDC3_VOLTAGE = 0x27;
static const uint8_t AXP192_LDO23_VOLTAGE = 0x28;

static const uint8_t AXP192_VBUS_IPSOUT_CHANNEL = 0x30;
static const uint8_t AXP192_SHUTDOWN_VOLTAGE = 0x31;
static const uint8_t AXP192_SHUTDOWN_BATTERY_CHGLED_CONTROL = 0x32;
static const uint8_t AXP192_CHARGE_CONTROL_1 = 0x33;
static const uint8_t AXP192_CHARGE_CONTROL_2 = 0x34;
static const uint8_t AXP192_BATTERY_CHARGE_CONTROL = 0x35;
static const uint8_t AXP192_PEK = 0x36;
static const uint8_t AXP192_DCDC_FREQUENCY = 0x37;
static const uint8_t AXP192_BATTERY_CHARGE_LOW_TEMP_ALARM = 0x38;
static const uint8_t AXP192_BATTERY_CHARGE_HIGH_TEMP_ALARM = 0x39;
static const uint8_t AXP192_APS_LOW_POWER1 = 0x3a;
static const uint8_t AXP192_APS_LOW_POWER2 = 0x3b;
static const uint8_t AXP192_BATTERY_DISCHARGE_LOW_TEMP_ALARM = 0x3c;
static const uint8_t AXP192_BATTERY_DISCHARGE_HIGH_TEMP_ALARM = 0x3d;
static const uint8_t AXP192_DCDC_MODE = 0x80;
static const uint8_t AXP192_ADC_ENABLE_1 = 0x82;
static const uint8_t AXP192_ADC_ENABLE_2 = 0x83;
static const uint8_t AXP192_ADC_RATE_TS_PIN = 0x84;
static const uint8_t AXP192_GPIO30_INPUT_RANGE = 0x85;
static const uint8_t AXP192_GPIO0_ADC_IRQ_RISING = 0x86;
static const uint8_t AXP192_GPIO0_ADC_IRQ_FALLING = 0x87;
static const uint8_t AXP192_TIMER_CONTROL = 0x8a;
static const uint8_t AXP192_VBUS_MONITOR = 0x8b;
static const uint8_t AXP192_TEMP_SHUTDOWN_CONTROL = 0x8f;

// Group 2 GPIO control type registers
static const uint8_t AXP192_GPIO0_CONTROL = 0x90;
static const uint8_t AXP192_GPIO0_LDO_VOLTAGE = 0x91;
static const uint8_t AXP192_GPIO1_CONTROL = 0x92;
static const uint8_t AXP192_GPIO2_CONTROL = 0x93;
static const uint8_t AXP192_GPIO20_SIGNAL_STATUS = 0x94;
static const uint8_t AXP192_GPIO43_FUNCTION_CONTROL = 0x95;
static const uint8_t AXP192_GPIO43_SIGNAL_STATUS = 0x96;
static const uint8_t AXP192_GPIO20_PULLDOWN_CONTROL = 0x97;
static const uint8_t AXP192_PWM1_FREQUENCY = 0x98;
static const uint8_t AXP192_PWM1_DUTY_CYCLE_1 = 0x99;
static const uint8_t AXP192_PWM1_DUTY_CYCLE_2 = 0x9a;
static const uint8_t AXP192_PWM2_FREQUENCY = 0x9b;
static const uint8_t AXP192_PWM2_DUTY_CYCLE_1 = 0x9c;
static const uint8_t AXP192_PWM2_DUTY_CYCLE_2 = 0x9d;
static const uint8_t AXP192_GPIO5_N_RSTO_CONTROL = 0x9e;

// Group 3, Interrupt control class registers 
static const uint8_t AXP192_ENABLE_CONTROL_1 = 0x40;
static const uint8_t AXP192_ENABLE_CONTROL_2 = 0x41;
static const uint8_t AXP192_ENABLE_CONTROL_3 = 0x42;
static const uint8_t AXP192_ENABLE_CONTROL_4 = 0x43;
static const uint8_t AXP192_ENABLE_CONTROL_5 = 0x4a;
static const uint8_t AXP192_IRQ_STATUS_1 = 0x44;
static const uint8_t AXP192_IRQ_STATUS_2 = 0x45;
static const uint8_t AXP192_IRQ_STATUS_3 = 0x46;
static const uint8_t AXP192_IRQ_STATUS_4 = 0x47;
static const uint8_t AXP192_IRQ_STATUS_5 = 0x4d;

// Group 4, ADC data category registers
static const uint8_t AXP192_ACIN_VOLTAGE = 0x56;
static const uint8_t AXP192_ACIN_CURRENT = 0x58;
static const uint8_t AXP192_VBUS_VOLTAGE = 0x5a;
static const uint8_t AXP192_VBUS_CURRENT = 0x5c;
static const uint8_t AXP192_TEMP = 0x5e;
static const uint8_t AXP192_TS_INPUT = 0x62;
static const uint8_t AXP192_GPIO0_VOLTAGE = 0x64;
static const uint8_t AXP192_GPIO1_VOLTAGE = 0x66;
static const uint8_t AXP192_GPIO2_VOLTAGE = 0x68;
static const uint8_t AXP192_GPIO3_VOLTAGE = 0x6a;
static const uint8_t AXP192_BATTERY_POWER = 0x70;
static const uint8_t AXP192_BATTERY_VOLTAGE = 0x78;
static const uint8_t AXP192_CHARGE_CURRENT = 0x7a;
static const uint8_t AXP192_DISCHARGE_CURRENT = 0x7c;
static const uint8_t AXP192_APS_VOLTAGE = 0x7e;

static const uint8_t AXP192_CHARGE_COULOMB = 0xb0;
static const uint8_t AXP192_DISCHARGE_COULOMB = 0xb4;
static const uint8_t AXP192_COULOMB_COUNTER_CONTROL = 0xb8;

}
}