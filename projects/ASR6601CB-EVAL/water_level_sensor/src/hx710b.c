#include "hx710b.h"
#include "tremo_gpio.h"
#include "lora_config.h"

enum HX_MODE { NONE, DIFF_10Hz, TEMP_40Hz, DIFF_40Hz};
float RES = 2.98023e-7;

//Private functions
static uint8_t hx710b_shiftInByte() {
    uint8_t value = 0;
    uint8_t i;

    for(i = 0; i < 8; ++i) {
        gpio_write(GPIOA, CONFIG_WATER_SENSOR_HX710B_SCK_PIN, GPIO_LEVEL_HIGH);
        //Read MSBFIRST
        value |= gpio_read(GPIOA, CONFIG_WATER_SENSOR_HX710B_OUT_PIN) << (7 - i);
        gpio_write(GPIOA, CONFIG_WATER_SENSOR_HX710B_SCK_PIN, GPIO_LEVEL_LOW);
    }
    return value;
}

static uint32_t readHX(uint8_t HX_MODE) {

  // pulse clock line to start a reading
  for (char i = 0; i < HX_MODE; i++) {
    gpio_write(GPIOA, CONFIG_WATER_SENSOR_HX710B_SCK_PIN, GPIO_LEVEL_HIGH);
    gpio_write(GPIOA, CONFIG_WATER_SENSOR_HX710B_SCK_PIN, GPIO_LEVEL_LOW);
  }

  // wait for the reading to finish
  while (gpio_read(GPIOA, CONFIG_WATER_SENSOR_HX710B_OUT_PIN)) {}

  // read the 24-bit pressure as 3 bytes using SPI
  uint8_t data[3];
  for (uint8_t j = 3; j--;) {
    data[j] = hx710b_shiftInByte();
  }

  gpio_write(GPIOA, CONFIG_WATER_SENSOR_HX710B_SCK_PIN, GPIO_LEVEL_HIGH);
  data[2] ^= 0x80;  // see note

  // shift the 3 bytes into a large integer
  uint32_t result = 0;
  result += (long)data[2] << 16;
  result += (long)data[1] << 8;
  result += (long)data[0];

  gpio_write(GPIOA, CONFIG_WATER_SENSOR_HX710B_SCK_PIN, GPIO_LEVEL_LOW);

  return result;
}

static float HX710B2pascal(uint32_t val){
    float value = ((val*RES) *200) + 500;
    return value;
}

//Public functions
float hx710b_read_water_cm()
{
  uint32_t val = readHX(DIFF_10Hz);
  float pascal = HX710B2pascal(val);
  float cm = 0.685*pascal - 600;
  return cm;
}

uint32_t hx710b_read_pressure_raw()
{
    return readHX(DIFF_10Hz);
}

