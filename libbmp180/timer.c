/* timer.c: Timing-related functions and syscalls.
 *
 *  Author: Arun Drelich
 *
 */

#include <signal.h>
#include <time.h>
#include "common.h"

#define MICROS_IN_MILLIS 1000000
#define NANOS_IN_MICROS 1000

/* Suspend execution of the calling thread and wait for the specified number
   of microseconds.
   Returns 0 on success and -1 on error. Errno is set accordingly.
*/
int wait_us(long delay_us) {
  struct timespec timeout;
  timeout.tv_sec = delay_us/MICROS_IN_MILLIS;
  timeout.tv_nsec = (delay_us%MICROS_IN_MILLIS)*NANOS_IN_MICROS;

  sigset_t empty;
  if (sigemptyset(&empty) == -1) {
    return -1;
  }

  return sigtimedwait(&empty, NULL, &timeout);
}

