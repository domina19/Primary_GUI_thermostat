#include <Arduino.h>
#include "supla_eeprom.h"
#include "thermostat.h"

#define SUPLADEVICE_CPP
#include <SuplaDevice.h>

_thermostat thermostat;

extern "C" {
#include "user_interface.h"
}

void thermostat_start() {
  pinMode(PIN_THERMOSTAT, OUTPUT);
  thermostatOFF();
  thermostat.temp = read_thermostat_temp();
  thermostat.hyst = read_thermostat_hyst();
  thermostat.channel = read_thermostat_channel();
  thermostat.channelSensor = 1;
  thermostat.error = 0;
}

bool CheckTermostat(int channelNumber, double temp) {
  double pom;
  if (channelNumber == thermostat.channel ) {
    if (temp == -275) {
      thermostat.error++;
      Serial.println("error");
      if (thermostat.error == 10) {
        thermostat.error = 0;
        thermostatOFF();
      }
      return false;
    }
    Serial.println("pomiar");
    pom = thermostat.temp - thermostat.hyst;
    if (thermostat.lower_temp) {
      if (temp > thermostat.temp) {
        thermostatOFF();
        Serial.println("Wyłącz przekaźnik - osiognięto gorny prób temperatury");
        return true;
      }
    }
    if (thermostat.upper_temp) {
      if (temp < pom) {
        if (thermostat.last_state == 1 ) {
          thermostatON();
          Serial.println("WŁĄCZ przekaźnik - osopgnięto dolny próg temperatury");
        }
        return true;
      }
    }
  }
  return false; //powrot z bledem
}

bool thermostatOFF() {
  thermostat.lower_temp = 0;
  thermostat.upper_temp = 1; //osiągnięto górny próg temperatury
  digitalWrite(PIN_THERMOSTAT, THERMOSTAT_OFF);
  SuplaDevice.channelValueChanged(thermostat.channelSensor, 1);
};

bool thermostatON() {
  thermostat.lower_temp = 1; //osiągnięto dolny próg temperatury
  thermostat.upper_temp = 0;
  digitalWrite(PIN_THERMOSTAT, THERMOSTAT_ON);
  SuplaDevice.channelValueChanged(thermostat.channelSensor, 0);
};
