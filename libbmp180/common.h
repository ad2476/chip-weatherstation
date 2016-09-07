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

typedef enum { OSS_LOW, OSS_STANDARD, OSS_HIGH, OSS_ULTRA } Oversample;

typedef struct bmp180_i2c {
  int fd; // i2c device fd
  /* calibration coef. reg. vals: */
  int16_t AC1, AC2, AC3; uint16_t AC4, AC5, AC6;
  int16_t B1, B2;
  int16_t MB, MC, MD;
  Oversample oss; // precision val used for measurements/calibration
  /* results of raw temp/pressure reads: */
  int32_t raw_temp_adc, raw_pressure_adc;
  float calib_temp_c, calib_press_hpa;
} bmp180_i2c_t;

// i2c-comms related:
extern int bmp180_init(struct bmp180_i2c *bmp, int i2cbus, Oversample precision);
extern int bmp180_close(struct bmp180_i2c *bmp);
extern uint8_t bmp180_checkID(struct bmp180_i2c *bmp);
extern int bmp180_pollADCvals(struct bmp180_i2c *bmp);

// calibration/conversion related:
extern void bmp180_compute_true_atmo(struct bmp180_i2c *bmp);

// timing-related:
extern int wait_us(long delay_ms);

#endif
