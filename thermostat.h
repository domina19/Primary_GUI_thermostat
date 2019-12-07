#ifndef THERMOSTAT_H
#define THERMOSTAT_H

#define THERMOSTAT_ON HIGH
#define THERMOSTAT_OFF LOW

#define TYPE_SENSOR_DS18B20  0
#define TYPE_SENSOR_DHT      1

#define THERMOSTAT_WARMING    0
#define THERMOSTAT_COOLLING   1
#define THERMOSTAT_HUMIDITY   2

typedef struct {
  double temp;
  double hyst;
  uint8_t humidity;
  uint8_t type;
  uint8_t typeSensor;
  uint8_t invertRelay;
  uint8_t channelDs18b20;
  uint8_t channelAuto;
  uint8_t channelManual;
  uint8_t channelSensor;
  uint8_t last_state_auto;
  uint8_t last_state_manual;
  uint8_t last_set_temp;
  uint8_t error;
} _thermostat;
extern _thermostat thermostat;

void thermostat_start(void);
void CheckTermostat(int channelNumber, double temp, double humidity);
void CheckTermostatWarming(double temp);
void CheckTermostatCooling(double temp);
void CheckTermostatHumidity(double humidity);
bool thermostatOFF();
bool thermostatON();
void valueChangeTemp();

#endif //THERMOSTAT_H
