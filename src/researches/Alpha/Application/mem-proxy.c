#include "mem-proxy.h"

#include <string.h>



static uint32_t addr_carret = 0;
static int buffer_carret = 0;
static bus_t * bus = 0;
static bool locked = false;

static uint8_t buffer[512] = { 0 };


static void commit_block();


int memproxy_init(bus_t * bus_)
{
	// TODO: Пройти по памяти и найти свободную страницу
	buffer[0] = 0x00;
	buffer_carret = 1;
	bus = bus_;

	bool found = false;
	for (uint32_t addr = 0x0000; addr <= 0xFF00; addr += 0x100)
	{
		uint8_t lead;
		mx25l512_read(bus, &addr, &lead, 0x01);
		if (lead != 0x00)
		{
			addr_carret = addr;
			found = true;
			break;
		}
	}

	if (!found)
		locked = true;

	return !locked;
}


int memproxy_write(const uint8_t * data, size_t size)
{
	if (locked)
		return 0;

	if (size > 0x100 - 1)
		return -1;

	memcpy(buffer+buffer_carret, data, size);
	buffer_carret += size;

	if (buffer_carret >= 0x100)
		commit_block();

	return 0;
}


static void commit_block()
{
	if (locked)
		return;

	if (buffer_carret < 0x100)
		return;

	mx25l512_PP_up(bus, &addr_carret, buffer, 0x100, 100);
	addr_carret += 0x100;
	if (addr_carret > 0xFF00)
	{
		addr_carret = 0;
		locked = true;
	}

	buffer[0] = 0x00;
	memmove(buffer+1, buffer+0x100, 0x100-1);
	buffer_carret -= 0x100-1;
}
