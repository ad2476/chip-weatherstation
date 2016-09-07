#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <libbmp180.h>

int main(int argc, char* argv[]) {
  if (argc != 2) {
    fprintf(stderr, "usage: %s [i2cdevice]\n", argv[0]);
    fprintf(stderr, "  i2cdevice: e.g. 2\n");
    return 1;
  }

  // initialise the device and populate struct:
  int device = atoi(argv[1]);
  bmp180_i2c_t bmp;
  int bmpfd = bmp180_init(&bmp, device, OSS_ULTRA);
  if (bmpfd == -1) {
    perror("bmp180_init");
    return 1;
  }

  // get the device ID:
  uint8_t bmp_ID = bmp180_checkID(&bmp);
  if (bmp_ID == BMP_INVALRES) {
    fprintf(stderr, "failed to read valid ID\n");
    return 1;
  }

  // Check it worked:
  printf("Device ID: %x\n", bmp_ID);

  uint16_t* baseptr = (uint16_t *)&bmp.AC1;
  printf("Calibration coefs:\n");
  for (int i = 0; i < 11; i++) {
    printf("  0x%04x\n", baseptr[i]);
  }

  while(1) {
    bmp180_pollADCvals(&bmp); // perform measurement reading
    bmp180_compute_true_atmo(&bmp); // populate temp and pressure fields
    printf("Temperature: %.02f deg. C\n", bmp.calib_temp_c);
    printf("Pressure: %.02f hPa\n", bmp.calib_press_hpa);
    wait_us(1000000);
  }

  bmp180_close(&bmp);
  return 0;
}
