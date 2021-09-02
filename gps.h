// gps.h

#include <stdio.h>
#include <stdint.h>

#define GPSBUFSIZE  128       // GPS buffer size

extern float latitude;
extern float longitude;

extern uint8_t rx_data;
extern uint8_t rx_buffer[GPSBUFSIZE];
extern uint8_t rx_index;
extern uint8_t parse_err;

//void GPS_Init();
void GPS_print_val(char *data, int value);
// void GPS_UART_CallBack();
int GPS_validate(char *nmeastr);
void GPS_parse(char *GPSstrParse);
float GPS_nmea_to_dec(float deg_coord, char nsew);
