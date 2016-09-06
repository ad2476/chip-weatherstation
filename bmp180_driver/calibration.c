/* calibration.c: Calibration code to convert raw temp or pressure data to
 *                calibrated unit values.
 *
 *  Author: Arun Drelich
 *
 */
#include "bmp180.h"

double computeTrueTemp(struct bmp180_i2c *bmp, int16_t raw_temp) {
  long x1 = ((raw_temp - bmp->AC6)*bmp->AC5)>>0xF;
  long x2 = (bmp->MC << 0xB)/(x1 + bmp->MD);
  long b5 = x1 + x2;

  long t_centiC = (b5 + 8) >> 4;
  return (float)t_centiC/10.0;
}

double computeTruePressure(struct bmp180_i2c *bmp, int16_t raw_pressure) {
  return 0.0;
}

