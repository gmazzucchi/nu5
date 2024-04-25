#ifndef TINYUSB_PEDALINATOR_PORTING_H
#define TINYUSB_PEDALINATOR_PORTING_H

#include "pedalinator_config.h"

#ifdef PEDALINATOR_COM_VIRTUAL_PORT_EXAMPLE
void led_blinking_task(void);
void cdc_task(void);
#endif

#endif  // TINYUSB_PEDALINATOR_PORTING_H
