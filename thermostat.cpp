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

  PIN_THERMOSTAT = read_gpio(0);
  PIN_THERMOMETR = read_gpio(1);
  LED_CONFIG_PIN = read_gpio(2);
  CONFIG_PIN = read_gpio(3);

  int val_button_auto = read_gpio(4);
  PIN_BUTTON_AUTO = (val_button_auto == 16 ? -1 : val_button_auto);

  int val_button_manual = read_gpio(5);
  PIN_BUTTON_MANUAL = (val_button_manual == 16 ? -1 : val_button_manual);

  thermostat.hyst = read_thermostat_hyst();
  thermostat.typeSensor = read_type_sensor();
  thermostat.type = read_thermostat_type();
  thermostat.invertRelay = read_invert_relay();

  thermostat.humidity = read_thermostat_humidity();
  thermostat.temp = read_thermostat_temp();
  if (chackThermostatHumidity()) {
    save_temp = thermostat.humidity;
  } else {
    save_temp = thermostat.temp;
  }

  MAX_DS18B20 = read_thermostat_max_ds();

  if (MAX_DS18B20 != 1) {
    thermostat.channelDs18b20 = read_thermostat_channel();
  } else {
    thermostat.channelDs18b20 = 0;
  }

  thermostat.channelAuto = 0;
  thermostat.channelManual = 1;
  thermostat.channelSensor = 4;
  thermostat.error = 0;
  thermostat.last_state_auto = 1;
  thermostat.last_state_manual = 0;
  thermostat.last_set_temp;

  thermostatOFF();
  pinMode(PIN_THERMOSTAT, OUTPUT);

  if (thermostat.type == THERMOSTAT_PWM || thermostat.type == THERMOSTAT_PWM_HUMIDITY) {
    analogWriteRange(100); // to have a range 1 - 100 for the fan
    analogWriteFreq(10000);
  }
}

void CheckTermostat(int channelNumber, double temp, double humidity) {
  if (thermostat.last_state_auto) {
    if (thermostat.typeSensor == TYPE_SENSOR_UNSET) return;
    if (channelNumber == thermostat.channelDs18b20 || thermostat.typeSensor == TYPE_SENSOR_DHT) {

      Serial.print("channel->"); Serial.print(channelNumber);

      if (thermostat.type == THERMOSTAT_WARMING ) {
        CheckTermostatWarming(temp);
        //CheckTermostatWarming(temp);
      } else if (thermostat.type == THERMOSTAT_COOLLING) {
        CheckTermostatCooling(temp);
      } else if (thermostat.type == THERMOSTAT_HUMIDITY) {
        CheckTermostatHumidity(humidity);
      } else if (thermostat.type == THERMOSTAT_PWM) {
        CheckTermostatPWM(temp);
      } else if (thermostat.type == THERMOSTAT_PWM_HUMIDITY) {
        CheckTermostatHumidityPWM(humidity);
      }
    }
  }
}

void CheckTermostatWarming(double temp) {
  double pom;
  if (temp == -275) {
    thermostat.error++;
    Serial.println("->error");
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
    Serial.println("->error");
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

  Serial.print("->Wilgotność - pomiar "); Serial.println(humidity);
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

void CheckTermostatPWM(double temp) {
  double pom;
  int fanSpeedPercent;

  if (temp == -275) {
    thermostat.error++;
    Serial.println("->error");
    if (thermostat.error == 10) {
      thermostat.error = 0;
      fanSpeedPercent = 0;
      controlFanSpeed(fanSpeedPercent);
      //thermostatOFF();
    }
    return;
  }
  Serial.print("->Grzanie PWM-pomiar "); Serial.println(temp);

  pom = thermostat.temp - thermostat.hyst ;

  if (temp < pom) {
    fanSpeedPercent = 100;
    relayStatus = 1;
  } else if ( temp >= pom && temp <= thermostat.temp) {
    fanSpeedPercent = map(temp, thermostat.temp, pom, 10, 100);
    relayStatus = 1;
  } else if (temp > thermostat.temp) {
    fanSpeedPercent = 0;
    relayStatus = 0;
  }
  controlFanSpeed(fanSpeedPercent);
}

void CheckTermostatHumidityPWM(double humidity) {
  double pom;
  int fanSpeedPercent;

  if (humidity == -1) {
    thermostat.error++;
    Serial.println("->error");
    if (thermostat.error == 10) {
      thermostat.error = 0;
      fanSpeedPercent = 0;
      controlFanSpeed(fanSpeedPercent);
      //thermostatOFF();
    }
    return;
  }
  Serial.print("->Wilgotność PWM-pomiar "); Serial.println(humidity);

  pom = thermostat.humidity - thermostat.hyst ;

  if (humidity < pom ) {
    fanSpeedPercent = 0;
    relayStatus = 0;
  } else if ( humidity >= pom && humidity <= thermostat.humidity ) {
    fanSpeedPercent = map(humidity, pom, thermostat.humidity, 10, 100);
    relayStatus = 1;
  } else if (humidity > thermostat.humidity) {
    fanSpeedPercent = 100;
    relayStatus = 1;
  }
  controlFanSpeed(fanSpeedPercent);
}

void controlFanSpeed (int fanSpeedPercent) {
  Serial.print("Fan Speed: ");
  Serial.println(fanSpeedPercent);
  analogWrite(PIN_THERMOSTAT, fanSpeedPercent); // set the fan speed
}

bool thermostatOFF() {
  relayStatus = 0;
  SuplaDevice.channelValueChanged(thermostat.channelSensor, 1);

  digitalWrite(PIN_THERMOSTAT, thermostat.invertRelay ? HIGH : LOW);

};

bool thermostatON() {
  relayStatus = 1;
  SuplaDevice.channelValueChanged(thermostat.channelSensor, 0);

  digitalWrite(PIN_THERMOSTAT, thermostat.invertRelay ? LOW : HIGH);

};

void valueChangeTemp() {
  if (chackThermostatHumidity()) {
    SuplaDevice.channelDoubleValueChanged(3, thermostat.humidity);
  } else {
    SuplaDevice.channelDoubleValueChanged(3, thermostat.temp);
  }
}

bool chackThermostatHumidity() {
  return thermostat.type == THERMOSTAT_HUMIDITY || thermostat.type == THERMOSTAT_PWM_HUMIDITY;
}
