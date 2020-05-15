#include <DallasTemperature.h>
#include "supla_eeprom.h"

#ifndef SUPLA_SETTINGS_H
#define SUPLA_SETTINGS_H


#define  SUPLA_DEVICE_NAME     "SUPLA TESTY"
#define  CONFIG_WIFI_LOGIN     "MODUL SUPLA"
#define  CONFIG_WIFI_PASSWORD  ""

#define  DEFAULT_LOGIN         "admin"
#define  DEFAULT_PASSWORD      "password"
#define  DEFAULT_HOSTNAME      "Primary termostat"

#define  UPDATE_PATH           "/firmware"

//DHT ****************************************************************************************************
#define  DHTTYPE               DHT22
#define  MAX_DHT               8

//DS18B20 ************************************************************************************************

extern uint8_t MAX_DS18B20; //maksymalnie 8
#define MAX_DS18B20_ARR 8
#define  TEMPERATURE_PRECISION  12  // rozdzielczość czujnika DS 9 -12 bit
#define TEMPERATURE_NOT_AVAILABLE -275

//LED CONFIG *********************************************************************************************
extern uint8_t LED_CONFIG_PIN;
//CONFIG PIN *********************************************************************************************
extern uint8_t CONFIG_PIN; //D3     // triger config
//GPIO
#define MAX_GPIO 6
//THERMOSTAT**********************************************************************************************
extern uint8_t PIN_BUTTON_AUTO;
extern uint8_t PIN_BUTTON_MANUAL;
extern uint8_t PIN_THERMOSTAT;
extern uint8_t PIN_THERMOMETR;
#define VIRTUAL_PIN_THERMOSTAT_AUTO 99
#define VIRTUAL_PIN_THERMOSTAT_MANUAL 98
#define VIRTUAL_PIN_SENSOR_THERMOSTAT 97
#define VIRTUAL_PIN_SET_TEMP 96
#define VIRTUAL_PIN_THERMOMETR 95

extern double save_temp;
//EEPROM *************************************************************************************************
#define EEPROM_SIZE           4096/4

#define  MAX_SSID            32
#define  MAX_PASSWORD        64
#define  MAX_MLOGIN          32
#define  MAX_MPASSWORD       64
#define  MIN_PASSWORD        8
#define  MAX_SUPLA_SERVER    SUPLA_SERVER_NAME_MAXSIZE
#define  MAX_SUPLA_ID        32
#define  MAX_SUPLA_PASS      SUPLA_LOCATION_PWD_MAXSIZE
#define  MAX_HOSTNAME        32
#define  MAX_BUTTON          16
#define  MAX_RELAY           16
#define  MAX_RELAY_STATE     16
#define  MAX_DS18B20_EEPROM  16
#define  MAX_DS18B20_SIZE    128
#define  MAX_GPIO_SIZE       16

#define  GUI_BLUE              "#005c96"
#define  GUI_GREEN             "#00D151"

#define CHOICE_TYPE  -1

extern char GUID[];
String read_rssi(void);
void supla_led_blinking(int led, int time);
void supla_led_blinking_stop(void);


void add_Sensor(int sensor);
void add_SensorNO(int sensor);
void add_Led_Config(int led);
void add_Config(int pin);
void add_Relay(int relay);
void add_Relay_Invert(int relay);
void add_DHT11_Thermometer(int thermpin);
void add_DHT22_Thermometer(int thermpin);
void add_DS18B20_Thermometer(int thermpin);
void add_Relay_Button(int relay, int button, int type);
void add_Relay_Button_Invert(int relay, int button, int type);
void add_DS18B20Multi_Thermometer(int thermpin);

double get_temperature(int channelNumber, double last_val);
void get_temperature_and_humidity(int channelNumber, double *temp, double *humidity);
String GetAddressToString(DeviceAddress deviceAddress);
extern double temp_html;
extern double humidity_html;
#endif //SUPLA_SETTINGS_H
