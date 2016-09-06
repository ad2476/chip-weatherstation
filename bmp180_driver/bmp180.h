/* bmp180.h: Header file for including any bmp180-related functions,defines,etc
 *
 *  Author: Arun Drelich
 *
 */

#ifndef _BMP180_H_
#define _BMP180_H_

#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#define I2C_ADDR 0x77

#define BMP_INVALRES ((1<<0xF)>>0xF) // i.e. 0b 1111 1111 1111 1111 i.e. -1

typedef struct bmp180_i2c {
  int fd; // i2c device fd
  /* calibration coef. reg. vals: */
  int16_t AC1, AC2, AC3; uint16_t AC4, AC5, AC6;
  int16_t B1, B2;
  int16_t MB, MC, MD;
  /* results of raw temp/pressure reads: */
  int16_t raw_temp_adc, raw_pressure_adc;
} bmp180_i2c_t;

typedef enum { OSS_LOW, OSS_STANDARD, OSS_HIGH, OSS_ULTRA } Oversample;

// i2c-comms related:
int bmp180_init(struct bmp180_i2c *bmp, int i2cbus);
int bmp180_close(struct bmp180_i2c *bmp);
uint8_t bmp180_checkID(struct bmp180_i2c *bmp);
int bmp180_pollADCvals(struct bmp180_i2c *bmp, Oversample precision);

// calibration/conversion related:
double computeTrueTemp(struct bmp180_i2c *bmp, int16_t raw_temp);
double computeTruePressure(struct bmp180_i2c *bmp, int16_t raw_pressure);

// timing-related:
int wait_us(long delay_ms);

#endif
