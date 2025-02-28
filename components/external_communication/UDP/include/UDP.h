#ifndef __UDP_H
#define __UDP_H

#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

extern void UDP_write_rc(const uint8_t *data, uint8_t len2);
extern void UDP_write_anotc(const uint8_t *data, uint8_t len2);
extern void UDP_init(void);
#endif // __UDP_H