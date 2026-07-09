#ifndef __FIBOCOM_H
#define __FIBOCOM_H

#include <stdint.h>

void fibocom_startup_test(void);
void fibocom_report_property(uint16_t type, uint16_t num);
void fibocom_voice_prompt(const char *text);
void fibocom_report_action_done(uint16_t type, uint16_t num);
void hub(uint16_t type, uint16_t num);

#endif