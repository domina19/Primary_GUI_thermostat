#ifndef THERMOSTAT_H
#define THERMOSTAT_H

#define THERMOSTAT_ON HIGH
#define THERMOSTAT_OFF LOW

typedef struct {
  double temp;
  double hyst;
  unsigned char upper_temp;
  unsigned char lower_temp;
  uint8_t type;
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
void CheckTermostat(int channelNumber, double temp);
void CheckTermostatWarming(int channelNumber, double temp);
void CheckTermostatCooling(int channelNumber, double temp);
bool thermostatOFF();
bool thermostatON();

#endif //THERMOSTAT_H
