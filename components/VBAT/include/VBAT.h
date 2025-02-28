#ifndef __VBAT_H
#define __VBAT_H

extern int VBAT;

void vbat_task(void *pvParameters);
extern int VBAT_init(void);
#endif // __VBAT_H