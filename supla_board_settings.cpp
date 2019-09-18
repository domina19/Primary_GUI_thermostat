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

extern "C" {
#include "user_interface.h"
}



void supla_board_configuration(void) {
//  if (Modul_tryb_konfiguracji == 2) return;
  
  MAX_GPIO = 4;
  add_Relay(VIRTUAL_PIN_THERMOSTAT_AUTO);
  SuplaDevice.addRelay(VIRTUAL_PIN_THERMOSTAT_MANUAL);
  SuplaDevice.addRelay(VIRTUAL_PIN_SET_TEMP);
  SuplaDevice.addDS18B20Thermometer();
  add_Sensor(VIRTUAL_PIN_SENSOR_THERMOSTAT);
  add_DS18B20Multi_Thermometer(PIN_THERMOMETR);
  //add_DHT22_Thermometer(PIN_THERMOMETR);

  add_Led_Config(LED_CONFIG_PIN);
  add_Config(CONFIG_PIN);

}
