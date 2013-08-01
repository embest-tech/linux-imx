#include <linux/slab.h>

#include "mmgr.h"

void *mmgr_zalloc(size_t size)
{
	return kzalloc(size, GFP_KERNEL);
}

void mmgr_free(const void *addr)
{
	kfree(addr);
}

