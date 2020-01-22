#include <MKL25Z4.H>
#include "gpio_defs.h"

void Delay (uint32_t dly) {
	
  volatile uint32_t t;
	SET_BIT(DEBUG_I2C_BUSY_WAIT);
  for (t=dly*10000; t>0; t--)
		;
	CLEAR_BIT(DEBUG_I2C_BUSY_WAIT);
}

void ShortDelay (uint32_t dly) {
  volatile uint32_t t;

	for (t=dly*10; t>0; t--)
		;
}
