/*
 * mem-proxy.h
 *
 *  Created on: Jun 29, 2023
 *      Author: Install
 */

#ifndef MEM_PROXY_H_
#define MEM_PROXY_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "MX25L512_/MX25L512_up.h"


int memproxy_init(bus_t * bus_);

int memproxy_write(const uint8_t * data, size_t size);

int memproxy_init_gps(bus_t * bus_);

int memproxy_write_gps(const uint8_t * data, size_t size);

#endif /* MEM_PROXY_H_ */
