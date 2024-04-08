#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>



float hx710b_read_water_cm();

uint32_t hx710b_read_pressure_raw();

float hx710b_read_pascal();

void hx710b_set_pd(uint8_t pd);

uint32_t hx710b_read_temperature(void);