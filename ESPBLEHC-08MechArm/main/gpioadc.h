#ifndef __gpioadc_h
#define __gpioadc_h

void adc_init(void);
void adc_task(void *arg);
void gpio_init(void);
void gpio_task(void *arg);
void get_btData(uint8_t *btData);
#endif
