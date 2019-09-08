#ifndef THERMOSTAT_H
#define THERMOSTAT_H

#define VIRTUAL_PIN_THERMOSTAT 99
#define VIRTUAL_PIN_SENSOR_THERMOSTAT 98
#define PIN_THERMOSTAT 4
#define THERMOSTAT_ON LOW
#define THERMOSTAT_OFF HIGH

typedef struct {
  double temp;
  double hyst;
  unsigned char upper_temp;
  unsigned char lower_temp;
  int channel;
  uint8_t last_state;
  byte error;
} _thermostat;
extern _thermostat thermostat;

void thermostat_start(void);
unsigned char CheckTermostat(int channelNumber, double temp);
bool thermostatOFF();
bool thermostatON();

#endif //THERMOSTAT_H
