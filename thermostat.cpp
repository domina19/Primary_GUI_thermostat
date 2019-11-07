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

  MAX_DS18B20 = read_thermostat_max_ds();
  thermostat.temp = read_thermostat_temp();
  save_temp = thermostat.temp;
  thermostat.hyst = read_thermostat_hyst();
  thermostat.humidity = read_thermostat_humidity();
  thermostat.typeSensor = read_type_sensor();
  thermostat.channelDs18b20 = read_thermostat_channel();
  thermostat.type = read_thermostat_type();
  thermostat.invertRelay = read_invert_relay();

  thermostat.channelAuto = 0;
  thermostat.channelManual = 1;
  thermostat.channelSensor = 4;
  thermostat.error = 0;
  thermostat.last_state_auto = 0;
  thermostat.last_state_manual = 0;
  thermostat.last_set_temp = 0;

  thermostatOFF();
  pinMode(PIN_THERMOSTAT, OUTPUT);
}

void CheckTermostat(int channelNumber, double temp, double humidity) {
  if (thermostat.last_state_auto) {
    if (channelNumber == thermostat.channelDs18b20 || thermostat.typeSensor == 1) {
      if (thermostat.type == 0 ) {
        Serial.print("channel-"); Serial.print(channelNumber);
        CheckTermostatWarming(temp);
      } else if (thermostat.type == 1) {
        Serial.print("channel-"); Serial.print(channelNumber);
        CheckTermostatCooling(temp);
      } else if (thermostat.type == 2) {
        CheckTermostatHumidity(humidity);
      }
    }
  }
}

void CheckTermostatWarming(double temp) {
  double pom;
  if (temp == -275) {
    thermostat.error++;
    Serial.println("error");
    if (thermostat.error == 10) {
      thermostat.error = 0;
      thermostatOFF();
    }
    return;
  }
  Serial.print("->Grzanie-pomiar "); Serial.println(temp);

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

void CheckTermostatCooling(double temp) {
  double pom;
  if (temp == -275) {
    thermostat.error++;
    Serial.println("error");
    if (thermostat.error == 10) {
      thermostat.error = 0;
      thermostatOFF();
    }
    return;
  }
  Serial.print("->Chłodzenie-pomiar "); Serial.println(temp);

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

void CheckTermostatHumidity(double humidity) {
  double pom;

  Serial.print("Wilgotność - pomiar "); Serial.println(humidity);
  pom = thermostat.humidity - thermostat.hyst ;

  if ( relayStatus == 1 && humidity < pom ) {
    thermostatOFF();
    Serial.println("OFF");
    return;
  }
  if (relayStatus == 0 && humidity >= thermostat.humidity) {
    thermostatON();
    Serial.println("ON");
    return;
  }
}

bool thermostatOFF() {
  relayStatus = 0;
  //digitalWrite(PIN_THERMOSTAT, THERMOSTAT_OFF);
  digitalWrite(PIN_THERMOSTAT, thermostat.invertRelay ? HIGH : LOW);
  SuplaDevice.channelValueChanged(thermostat.channelSensor, 1);
};

bool thermostatON() {
  relayStatus = 1;
  //digitalWrite(PIN_THERMOSTAT, THERMOSTAT_ON);
  digitalWrite(PIN_THERMOSTAT, thermostat.invertRelay ? LOW : HIGH);
  SuplaDevice.channelValueChanged(thermostat.channelSensor, 0);
};

void valueChangeTemp() {
  if (thermostat.type == 2) {
    SuplaDevice.channelDoubleValueChanged(3, thermostat.humidity);
  } else {
    SuplaDevice.channelDoubleValueChanged(3, thermostat.temp);
  }
}
