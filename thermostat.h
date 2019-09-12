#ifndef THERMOSTAT_H
#define THERMOSTAT_H

#define VIRTUAL_PIN_THERMOSTAT_AUTO 99
#define VIRTUAL_PIN_THERMOSTAT_MANUAL 98
#define VIRTUAL_PIN_SENSOR_THERMOSTAT 97
#define PIN_THERMOSTAT 4
#define THERMOSTAT_ON HIGH
#define THERMOSTAT_OFF LOW

typedef struct {
  double temp;
  double hyst;
  unsigned char upper_temp;
  unsigned char lower_temp;
  uint8_t channelDs18b20;
  uint8_t channelAuto;
  uint8_t channelManual;
  uint8_t channelSensor;
  uint8_t last_state_auto;
  uint8_t last_state_manual;
  uint8_t error;
} _thermostat;
extern _thermostat thermostat;

void thermostat_start(void);
bool CheckTermostat(int channelNumber, double temp);
bool thermostatOFF();
bool thermostatON();

#endif //THERMOSTAT_H
