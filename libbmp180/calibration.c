/* calibration.c: Calibration code to convert raw temp or pressure data to
 *                calibrated unit values.
 *
 *  Author: Arun Drelich
 *
 */
#include "bmp180.h"

/* Once the raw ADC values have been sampled and read from their
   respective registers, pass the bmp struct to this function, which will
   convert the raw values into human-understandable temperature and pressure
   units.
   Returns: Nothing, but updates the fields calib_temp_c and calib_press_hpa
            inside bmp
*/
void bmp180_compute_true_atmo(struct bmp180_i2c *bmp) {
  int32_t x1 = ((bmp->raw_temp_adc - bmp->AC6)*bmp->AC5)>>0xF;
  int32_t x2 = (bmp->MC << 0xB)/(x1 + bmp->MD);
  int32_t b5 = x1 + x2;

  int32_t t_centiC = (b5 + 8) >> 4;
  bmp->calib_temp_c = (float)t_centiC/10.0;

  int32_t b6 = b5 - 4000;
  x1 = bmp->B2*(b6*(b6>>0xC)) >> 0xB;
  x2 = (bmp->AC2 * b6) >> 0xB;
  int32_t x3 = x1 + x2;
  int32_t b3 = ((((bmp->AC1<<2) + x3)<<bmp->oss) + 2) >> 2;
  x1 = (bmp->AC3 * b6) >> 0xD;
  x2 = (bmp->B1*(b6*(b6>>0xC))) >> 0x10;
  x3 = ((x1+x2)+2)>>2;
  uint32_t b4 = bmp->AC4*(uint32_t)(x3+32768) >> 0xF;
  uint32_t b7 = ((uint32_t)bmp->raw_pressure_adc - b3)*(50000>>bmp->oss);
  int32_t p;
  if (b7 < (uint32_t)(1<<0xF)) {
    p = (b7<<1)/b4;
  } else {
    p = (b7/b4)<<1;
  }
  x1 = (((p<<8)*(p<<8))*3038)>>0x10;
  x2 = (-7357*p)>>0x10;
  p = p + ((x1+x2+3791)>>4); // pressure in pascal

  bmp->calib_press_hpa = (float)p/100.0;

}

