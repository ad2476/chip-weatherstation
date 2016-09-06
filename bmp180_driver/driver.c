/* driver.c: Implements communication with the BMP180 device over the I2C bus.
 *
 *  Author: Arun Drelich
 *
 */

#include "bmp180.h"
#include "registers.h"

#define DEVFN_SIZE 16

uint16_t bmp180_read_register_word(int fd, uint8_t addr);
uint8_t bmp180_read_register_byte(int fd, uint8_t addr);
int bmp180_write_register_byte(int fd, uint8_t reg, uint8_t val);

const long OSS_CONV_TIME[] = {4500, 7500, 13500, 25500};
const uint8_t TEMP_CTLREG_VAL = 0x2E;
const uint8_t PRESSURE_CTLREG_VALS[] = {0x34, 0x74, 0xB4, 0xF4};

/* Initialises communication with the i2c device and sets the fields of the
   passed struct. Returns fd on success.
   If there was an error, returns -1 with errno set appropriately
   by the syscall in which the error occurred. The structs fields are not
   then guaranteed to be initialised.
*/
int bmp180_init(struct bmp180_i2c *bmp, int i2cbus) {
  char i2c_devfn[DEVFN_SIZE];
  snprintf(i2c_devfn, DEVFN_SIZE, "/dev/i2c-%i", i2cbus);
  int fd;
  if ((fd = open(i2c_devfn, O_RDWR)) == -1) {
    return -1;
  }

  // Initialise communication to this device:
  if (ioctl(fd, I2C_SLAVE, I2C_ADDR) == -1) {
    return -1;
  }

  bmp->fd = fd;

  // read calibration coefs from EEPROM:
  bmp->AC1 = (int16_t)bmp180_read_register_word(fd,REG_AC1);
  bmp->AC2 = (int16_t)bmp180_read_register_word(fd,REG_AC2);
  bmp->AC3 = (int16_t)bmp180_read_register_word(fd,REG_AC3);
  bmp->AC4 = (uint16_t)bmp180_read_register_word(fd,REG_AC4);
  bmp->AC5 = (uint16_t)bmp180_read_register_word(fd,REG_AC5);
  bmp->AC6 = (uint16_t)bmp180_read_register_word(fd,REG_AC6);
  bmp->B1 = (int16_t)bmp180_read_register_word(fd,REG_B1);
  bmp->B2 = (int16_t)bmp180_read_register_word(fd,REG_B2);
  bmp->MB = (int16_t)bmp180_read_register_word(fd,REG_MB);
  bmp->MC = (int16_t)bmp180_read_register_word(fd,REG_MC);
  bmp->MD = (int16_t)bmp180_read_register_word(fd,REG_MD);

  return fd;
}

/* Get the value in the REG_ID register, mostly to test that i2c comms are
   actually working.
*/
uint8_t bmp180_checkID(struct bmp180_i2c *bmp) {
  return bmp180_read_register_byte(bmp->fd, REG_ID);
}

/* Poll the sensor for the raw (uncalibrated) temperature and pressure data,
   and store it back in the struct. Specify an oversampling precision level.
   Returns 0 on success, -1 on error.
*/
int bmp180_pollADCvals(struct bmp180_i2c *bmp, Oversample precision) {
  // write 0x2e to register 0xf4: do temp measurement
  if (bmp180_write_register_byte(bmp->fd,REG_CTRL_MEAS,TEMP_CTLREG_VAL) == -1) {
    return -1;
  }
  wait_us(4500); // wait time for temp is 4.5ms
  // read raw temp value
  uint16_t raw;
  if ((raw = bmp180_read_register_word(bmp->fd,REG_OUT_MSB)) == -1) {
    return -1;
  }
  bmp->raw_temp_adc = (int16_t)raw; // store raw temp value

  // write pressure-related command:
  uint8_t pressurectl = (precision<<6) | PRESSURE_CTLREG_VALS[precision];
  if (bmp180_write_register_byte(bmp->fd,REG_CTRL_MEAS,pressurectl) == -1) {
    return -1;
  }
  wait_us(OSS_CONV_TIME[precision]); // wait for us appropriate for precision
  // read raw pressure value:
  if ((raw = bmp180_read_register_word(bmp->fd,REG_OUT_MSB)) == -1) {
    return -1;
  }
  bmp->raw_pressure_adc = (int16_t)raw;

  return 0;
}

/* Initiate a request to read from a word register (16 bits wide) by writing
   the address `addr` to the bus. Returns -1 on error.
*/
uint16_t bmp180_read_register_word(int fd, uint8_t reg) {
  ssize_t bytes_written, bytes_read;

  // write register address to slave
  if ((bytes_written = write(fd, &reg, 1)) != 1) {
    return BMP_INVALRES;
  }

  // read response from slave into buffer
  uint8_t buf[2] = {0};
  if ((bytes_read = read(fd, &buf, 2)) != 2) {
    return BMP_INVALRES;
  }

  uint16_t res = (buf[0] << 8) | buf[1]; // separate statement for debugging
  return res;
}

/* Initiate a request to read a byte register (8 bits wide). Operates similarly
   to bmp180_read_register_word(). Returns -1 on error.
*/
uint8_t bmp180_read_register_byte(int fd, uint8_t reg) {
  ssize_t bytes_written, bytes_read;
  if ((bytes_written = write(fd, &reg, 1)) != 1) {
    return BMP_INVALRES;
  }
  uint8_t buf = 0;
  if ((bytes_read = read(fd, &buf, 1)) != 1) {
    return BMP_INVALRES;
  }
  return buf;
}

/* Write the byte value val to register reg. Returns -1 on error, number of
   bytes written on success
*/
int bmp180_write_register_byte(int fd, uint8_t reg, uint8_t val) {
  ssize_t bytes_written;
  uint8_t cmd[2] = {reg, val};
  return write(fd, &cmd, 2);
}

/* Close the fd opened by bmp180_init(). This just calls close() on the open fd.
*/
int bmp180_close(struct bmp180_i2c *bmp) {
  return close(bmp->fd);
}
