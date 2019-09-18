#include <Arduino.h>
#include "supla_eeprom.h"
#include "supla_settings.h"
#include "thermostat.h"

#define SUPLADEVICE_CPP
#include <SuplaDevice.h>

_thermostat thermostat;

extern "C" {
#include "user_interface.h"
}

int relayStatus;

void thermostat_start() {
  pinMode(PIN_THERMOSTAT, OUTPUT);
  thermostatOFF();
  thermostat.temp = read_thermostat_temp();
  thermostat.hyst = read_thermostat_hyst();
  thermostat.channelDs18b20 = read_thermostat_channel();
  thermostat.type = read_thermostat_type();

  thermostat.channelAuto = 0;
  thermostat.channelManual = 1;
  thermostat.channelSensor = 4;
  thermostat.error = 0;
  thermostat.last_state_auto = 0;
  thermostat.last_state_manual = 0;
  thermostat.last_set_temp = 0;

  //ustawienie temperatury termostatu na kanału
  SuplaDevice.channelDoubleValueChanged(3, thermostat.temp);
}

void CheckTermostat(int channelNumber, double temp) {
  if (thermostat.type) {
    CheckTermostatWarming(channelNumber, temp);
  } else {
    CheckTermostatCooling(channelNumber, temp);
  }
}

void CheckTermostatWarming(int channelNumber, double temp) {
  double pom;
  if (channelNumber == thermostat.channelDs18b20 && thermostat.last_state_auto) {
    if (temp == -275) {
      thermostat.error++;
      Serial.println("error");
      if (thermostat.error == 10) {
        thermostat.error = 0;
        thermostatOFF();
      }
      return;
    }
    Serial.print("Grzanie - pomiar "); Serial.println(temp);

    pom = thermostat.temp + thermostat.hyst ;

    if (relayStatus == 0 && temp <= thermostat.temp) {
      thermostatON();
      Serial.println("ON");
      return;
    }
    if ( relayStatus == 1 && temp > pom) {
      thermostatOFF();
      Serial.println("OFF");
      return;
    }
  }
}

void CheckTermostatCooling(int channelNumber, double temp) {
  double pom;
  if (channelNumber == thermostat.channelDs18b20 && thermostat.last_state_auto) {
    if (temp == -275) {
      thermostat.error++;
      Serial.println("error");
      if (thermostat.error == 10) {
        thermostat.error = 0;
        thermostatOFF();
      }
      return;
    }
    Serial.print("Chłodzenie - pomiar: "); Serial.println(temp);

    pom = thermostat.temp - thermostat.hyst ;

    if ( relayStatus == 1 && temp < pom ) {
      thermostatOFF();
      Serial.println("OFF");
      return;
    }
    if (relayStatus == 0 && temp >= thermostat.temp) {
      thermostatON();
      Serial.println("ON");
      return;
    }
  }
}

bool thermostatOFF() {
  relayStatus = 0;
  digitalWrite(PIN_THERMOSTAT, THERMOSTAT_OFF);
  SuplaDevice.channelValueChanged(thermostat.channelSensor, 1);
};

bool thermostatON() {
  relayStatus = 1;
  digitalWrite(PIN_THERMOSTAT, THERMOSTAT_ON);
  SuplaDevice.channelValueChanged(thermostat.channelSensor, 0);
};
