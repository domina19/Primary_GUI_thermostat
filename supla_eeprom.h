

#ifndef SUPLA_EEPROM_H
#define SUPLA_EEPROM_H

void Pokaz_zawartosc_eeprom();
void czyszczenieEepromAll(void);
void czyszczenieEeprom(void);

void save_wifi_ssid(String save);
void save_wifi_pass(String save);
void save_login(String save);
void save_login_pass(String save);
void save_supla_server(String save);
void save_supla_id(String save);
void save_supla_pass(String save);
void save_supla_hostname(String save);
void save_guid(void);
void save_supla_button_type(int nr, String save);
void save_supla_relay_flag(int nr, String save);
void save_supla_relay_state(int nr, String save);
void save_DS18b20_address(String save, int nr);
void save_gpio(int nr, String save);
void save_thermostat_temp(double temp);
void save_thermostat_hyst(double temp);
void save_thermostat_channel(uint8_t temp);
void save_thermostat_type(uint8_t save);
void save_thermostat_max_ds(uint8_t save);
void save_type_sensor(int save);
void save_thermostat_humidity(int save);
void save_invert_relay(int val);

String read_wifi_ssid(void);
String read_wifi_pass(void);
String read_login(void);
String read_login_pass(void);
String read_supla_server(void);
String read_supla_id(void);
String read_supla_pass(void);
String read_supla_hostname(void);
String read_guid(void);
int read_supla_button_type(int nr);
int read_supla_relay_flag(int nr);
int read_supla_relay_state(int nr);
String read_DS18b20_address(int nr);
int read_gpio(int nr);
double read_thermostat_temp(void);
double read_thermostat_hyst(void);
uint8_t read_thermostat_channel(void);
uint8_t read_thermostat_type(void);
uint8_t read_thermostat_max_ds(void);
int read_type_sensor();
int read_thermostat_humidity();
int read_invert_relay();
#endif //SUPLA_EEPROM_H
