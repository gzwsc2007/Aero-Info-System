/*
 * Readers-writer lock implemented using RT-Thread's
 * semaphores. Favors writer over readers as writes
 * are rarer and more important.
 *
 * Author: Anson Wang
 */

#include <rtthread.h>
#include "rwlock.h"

void rwlock_init(rwlock_t *lock) {
    lock->readCnt = 0;
    lock->wrWaitCnt = 0;
    if (rt_mutex_init(&lock->tempLock, "rdlock", RT_IPC_FLAG_FIFO) != RT_EOK)
        rt_kprintf("rt_mutex_init error\n");
    if (rt_sem_init(&lock->writeLock, "wrlock", 1, RT_IPC_FLAG_FIFO) != RT_EOK)
        rt_kprintf("rt_sem_init error\n");
    if (rt_sem_init(&lock->wrWaitLock, "wrWaitLock", 1, RT_IPC_FLAG_FIFO) != RT_EOK)
        rt_kprintf("rt_sem_init error\n");
}

void rwlock_rdlock(rwlock_t *lock) {
    rt_mutex_take(&lock->tempLock, RT_WAITING_FOREVER);
    
    // writer gets priority over readers
    if (lock->wrWaitCnt > 0) {
      rt_mutex_release(&lock->tempLock);
      rt_sem_take(&lock->wrWaitLock, RT_WAITING_FOREVER);
      rt_sem_release(&lock->wrWaitLock);
      rt_mutex_take(&lock->tempLock, RT_WAITING_FOREVER);
    }
    
    // first reader in, acquire writelock to prevent writer from writing
    if (lock->readCnt == 0) {
      rt_mutex_release(&lock->tempLock);
      rt_sem_take(&lock->writeLock, RT_WAITING_FOREVER);
      rt_mutex_take(&lock->tempLock, RT_WAITING_FOREVER);
    }
    lock->readCnt++;
    
    rt_mutex_release(&lock->tempLock);
}

void rwlock_rdunlock(rwlock_t *lock) {
    rt_mutex_take(&lock->tempLock, RT_WAITING_FOREVER);
    
    lock->readCnt--;
    // last out
    if (lock->readCnt == 0) {
      rt_sem_release(&lock->writeLock);
    }
    
    rt_mutex_release(&lock->tempLock);
}

void rwlock_wrlock(rwlock_t *lock) {
    rt_mutex_take(&lock->tempLock, RT_WAITING_FOREVER);
    
    if (lock->readCnt > 0) {
      lock->wrWaitCnt++;
      rt_mutex_release(&lock->tempLock);
      rt_sem_take(&lock->wrWaitLock, RT_WAITING_FOREVER);
    }
    else
      rt_mutex_release(&lock->tempLock);
  
    rt_sem_take(&lock->writeLock, RT_WAITING_FOREVER);
}

void rwlock_wrunlock(rwlock_t *lock) {
    rt_mutex_take(&lock->tempLock, RT_WAITING_FOREVER);

    if (lock->wrWaitCnt > 0) {
      lock->wrWaitCnt--;
      rt_mutex_release(&lock->tempLock);
      rt_sem_release(&lock->wrWaitLock);
    }
    else
      rt_mutex_release(&lock->tempLock);

    rt_sem_release(&lock->writeLock);
}