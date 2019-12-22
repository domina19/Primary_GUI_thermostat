
#ifndef SUPLA_BOARD_SETTINGS_H
#define SUPLA_BOARD_SETTINGS_H


extern int nr_button;
extern int nr_relay;
extern int nr_ds18b20;
extern int nr_dht;
extern int dht_channel[];

typedef struct {
  int relay;
  int invert;
} _relay_button_channel;
extern _relay_button_channel relay_button_channel[];

extern DallasTemperature sensor[];
typedef struct {
  int pin;
  int channel;
  String address;
  int type; //0-single 1-multi
  DeviceAddress deviceAddress;

} _ds18b20_t;
extern _ds18b20_t ds18b20_channel[];
void supla_board_configuration(void);


#endif //SUPLA_BOARD_SETTINGS_H
