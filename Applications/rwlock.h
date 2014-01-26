#ifndef _RWLOCK_H_
#define _RWLOCK_H_

typedef struct {
    rt_uint8_t readCnt;
    rt_uint8_t wrWaitCnt;
    struct rt_mutex tempLock;
    struct rt_semaphore writeLock;
    struct rt_semaphore wrWaitLock;
} rwlock_t;


void rwlock_init(rwlock_t *lock);

// apply reader lock
void rwlock_rdlock(rwlock_t *lock);

// reader unlock
void rwlock_rdunlock(rwlock_t *lock);

// apply writer lock
void rwlock_wrlock(rwlock_t *lock);

// unlock writer
void rwlock_wrunlock(rwlock_t *lock);

#endif