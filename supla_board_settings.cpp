#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>

#include <EEPROM.h>
#include <DoubleResetDetector.h> //Bilioteka by Stephen Denne

#define SUPLADEVICE_CPP
#include <SuplaDevice.h>

#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>

#include "supla_settings.h"
#include "supla_eeprom.h"
#include "supla_web_server.h"
#include "supla_board_settings.h"
#include "thermostat.h"

extern "C" {
#include "user_interface.h"
}

void supla_board_configuration(void) {

  //  if (Modul_tryb_konfiguracji == 2) return;
  add_Relay_Button(VIRTUAL_PIN_THERMOSTAT_AUTO, PIN_BUTTON_AUTO, INPUT_TYPE_BTN_MONOSTABLE);
  SuplaDevice.addRelayButton(VIRTUAL_PIN_THERMOSTAT_MANUAL, PIN_BUTTON_MANUAL, INPUT_TYPE_BTN_BISTABLE, RELAY_FLAG_RESET);
  SuplaDevice.addRelayButton(VIRTUAL_PIN_SET_TEMP, INPUT_TYPE_BTN_NONE, INPUT_TYPE_BTN_BISTABLE, RELAY_FLAG_RESET);
  SuplaDevice.addDS18B20Thermometer();
  add_Sensor(VIRTUAL_PIN_SENSOR_THERMOSTAT);

  if (thermostat.typeSensor == TYPE_SENSOR_DS18B20) {
    if (MAX_DS18B20 == 1) {
      add_DS18B20_Thermometer(PIN_THERMOMETR);
    } else {
      add_DS18B20Multi_Thermometer(PIN_THERMOMETR);
    }
  } else if (thermostat.typeSensor == TYPE_SENSOR_DHT) {
    add_DHT22_Thermometer(PIN_THERMOMETR);
  }
  // add_DHT22_Thermometer(14);
  add_Led_Config(LED_CONFIG_PIN);
  add_Config(CONFIG_PIN);
}
