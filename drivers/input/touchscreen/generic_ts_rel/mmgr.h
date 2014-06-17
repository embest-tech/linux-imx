
#ifndef MMGR_H
#define MMGR_H

void *mmgr_zalloc(size_t size);
void mmgr_free(const void *addr);

#endif
