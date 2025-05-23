/****************************************************************************
 * include/pthread.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __INCLUDE_PTHREAD_H
#define __INCLUDE_PTHREAD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>    /* Default settings */
#include <nuttx/compiler.h>  /* Compiler settings, noreturn_function */
#include <nuttx/mutex.h>

#include <sys/types.h>       /* Needed for general types */
#include <stdint.h>          /* C99 fixed width integer types */
#include <stdbool.h>         /* C99 boolean types */
#include <unistd.h>          /* For getpid */
#include <signal.h>          /* Needed for sigset_t, includes this file */
#include <time.h>            /* Needed for struct timespec */

#ifdef CONFIG_PTHREAD_SPINLOCKS
/* The architecture specific spinlock.h header file must provide the
 * following:
 *
 *   SP_LOCKED   - A definition of the locked state value (usually 1)
 *   SP_UNLOCKED - A definition of the unlocked state value (usually 0)
 *   spinlock_t  - The type of a spinlock memory object.
 *
 * SP_LOCKED and SP_UNLOCKED must constants of type spinlock_t.
 */

#  include <nuttx/spinlock_type.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Standard POSIX switches */

#ifndef _POSIX_THREADS
#  define _POSIX_THREADS
#endif

#ifndef _POSIX_THREAD_ATTR_STACKSIZE
#  define _POSIX_THREAD_ATTR_STACKSIZE
#endif

/* Values for the process shared (pshared) attribute */

#define PTHREAD_PROCESS_PRIVATE       0
#define PTHREAD_PROCESS_SHARED        1

/* Values for the mutext type attribute:
 *
 * PTHREAD_MUTEX_NORMAL: This type of mutex does not detect deadlock.
 *   A thread attempting to relock this mutex without first unlocking
 *   it will deadlock.  Attempting to unlock a mutex locked by a different
 *   thread results in undefined behavior.  Attempting to unlock an unlocked
 *   mutex results in undefined behavior.
 * PTHREAD_MUTEX_ERRORCHECK
 *   This type of mutex provides error checking. A thread attempting
 *   to relock this mutex without first unlocking it will return with
 *   an error.  A thread attempting to unlock a mutex which another thread
 *   has locked will return with an error.  A thread attempting to unlock
 *   an unlocked mutex will return with an error.
 * PTHREAD_MUTEX_RECURSIVE
 *   A thread attempting to relock this mutex without first unlocking it will
 *   succeed in locking the mutex. The relocking deadlock which can occur
 *   with mutexes of type PTHREAD_MUTEX_NORMAL cannot occur with this type
 *   of mutex.  Multiple locks of this mutex require the same number
 *   of unlocks to release the mutex before another thread can acquire
 *   the mutex.  A thread attempting to unlock a mutex which another thread
 *   has locked will return with an error.  A thread attempting to unlock
 *   an unlocked mutex will return with an error.
 * PTHREAD_MUTEX_DEFAULT
 *  An implementation is allowed to map this mutex to one of the other mutex
 *  types.
 */

#define PTHREAD_MUTEX_NORMAL          0
#define PTHREAD_MUTEX_ERRORCHECK      1
#define PTHREAD_MUTEX_RECURSIVE       2
#define PTHREAD_MUTEX_DEFAULT         PTHREAD_MUTEX_NORMAL

/* Valid ranges for the pthread stacksize attribute */

#define PTHREAD_STACK_MIN             CONFIG_PTHREAD_STACK_MIN
#define PTHREAD_STACK_DEFAULT         CONFIG_PTHREAD_STACK_DEFAULT

/* Values for the pthread inheritsched attribute */

#define PTHREAD_INHERIT_SCHED         0
#define PTHREAD_EXPLICIT_SCHED        1

/* Detach state  */

#define PTHREAD_CREATE_JOINABLE       0
#define PTHREAD_CREATE_DETACHED       1

/* Default priority */

#define PTHREAD_DEFAULT_PRIORITY      SCHED_PRIORITY_DEFAULT

/* Cancellation states used by pthread_setcancelstate() */

#define PTHREAD_CANCEL_ENABLE         (0)
#define PTHREAD_CANCEL_DISABLE        (1)

/* Cancellation types used by pthread_setcanceltype() */

#define PTHREAD_CANCEL_DEFERRED       (0)
#define PTHREAD_CANCEL_ASYNCHRONOUS   (1)

/* Thread return value when a pthread is canceled */

#define PTHREAD_CANCELED              ((FAR void*)ERROR)

/* Used to initialize a pthread_once_t */

#define PTHREAD_ONCE_INIT             {false, PTHREAD_MUTEX_INITIALIZER}

/* This is returned by pthread_barrier_wait.  It must not match any errno
 * in errno.h
 */

#define PTHREAD_BARRIER_SERIAL_THREAD 0x1000

/* Values for protocol mutex attribute */

#define PTHREAD_PRIO_NONE             SEM_PRIO_NONE
#define PTHREAD_PRIO_INHERIT          SEM_PRIO_INHERIT
#define PTHREAD_PRIO_PROTECT          SEM_PRIO_PROTECT

/* Values for robust argument of pthread_mutexattr_get/setrobust
 *
 * PTHREAD_MUTEX_STALLED - No special actions are taken if the owner of the
 * mutex is terminated while holding the mutex lock. This can lead to
 * deadlocks if no other thread can unlock the mutex.  This is the standard
 * default value (NuttX permits you to override that default behavior
 * with a configuration option).
 *
 * PTHREAD_MUTEX_ROBUST - If the process containing the owning thread of a
 * robust mutex terminates while holding the mutex lock, the next thread
 * that acquires the mutex will be notified about the termination by the
 * return value EOWNERDEAD from the locking function. If the owning thread
 * of a robust mutex terminates while holding the mutex lock, the next
 * thread that attempts to acquire the mutex may be notified about the
 * termination by the return value EOWNERDEAD. The notified thread can
 * then attempt to make the state protected by the mutex consistent again,
 * and if successful can mark the mutex state as consistent by calling
 * pthread_mutex_consistent(). After a subsequent successful call to
 * pthread_mutex_unlock(), the mutex lock will be released and can be used
 * normally by other threads. If the mutex is unlocked without a call to
 * pthread_mutex_consistent(), it will be in a permanently unusable state
 * and all attempts to lock the mutex will fail with the error
 * ENOTRECOVERABLE. The only permissible operation on such a mutex is
 * pthread_mutex_destroy().
 */

#define PTHREAD_MUTEX_STALLED         0
#define PTHREAD_MUTEX_ROBUST          1

/* Values for struct pthread_mutex_s flags.  These are non-standard and
 * intended only for internal use within the OS.
 */

#define _PTHREAD_MFLAGS_ROBUST        (1 << 0) /* Robust (NORMAL) mutex */
#define _PTHREAD_MFLAGS_INCONSISTENT  (1 << 1) /* Mutex is in an inconsistent state */
#define _PTHREAD_MFLAGS_NRECOVERABLE  (1 << 2) /* Inconsistent mutex has been unlocked */

/* The contention scope attribute in thread attributes object */

#define PTHREAD_SCOPE_SYSTEM          0
#define PTHREAD_SCOPE_PROCESS         1

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/* pthread-specific types */

#ifndef __PTHREAD_KEY_T_DEFINED
typedef int pthread_key_t;
#  define __PTHREAD_KEY_T_DEFINED 1
#endif

#ifndef __PTHREAD_ADDR_T_DEFINED
typedef FAR void *pthread_addr_t;
#  define __PTHREAD_ADDR_T_DEFINED 1
#endif

typedef CODE pthread_addr_t (*pthread_startroutine_t)(pthread_addr_t);
typedef pthread_startroutine_t pthread_func_t;

typedef void (*pthread_trampoline_t)(pthread_startroutine_t, pthread_addr_t);

struct pthread_attr_s
{
  uint8_t priority;            /* Priority of the pthread */
  uint8_t policy;              /* Pthread scheduler policy */
  uint8_t inheritsched;        /* Inherit parent priority/policy? */
  uint8_t detachstate;         /* Initialize to the detach state */
#ifdef CONFIG_SCHED_SPORADIC
  uint8_t low_priority;        /* Low scheduling priority */
  uint8_t max_repl;            /* Maximum pending replenishments */
#endif

#ifdef CONFIG_SMP
  cpu_set_t affinity;          /* Set of permitted CPUs for the thread */
#endif

  FAR void  *stackaddr;        /* Address of memory to be used as stack */
  size_t stacksize;            /* Size of the stack allocated for the pthread */

#ifdef CONFIG_SCHED_SPORADIC
  struct timespec repl_period; /* Replenishment period */
  struct timespec budget;      /* Initial budget */
#endif
};

#ifndef __PTHREAD_ATTR_T_DEFINED
typedef struct pthread_attr_s pthread_attr_t;
#  define __PTHREAD_ATTR_T_DEFINED 1
#endif

#ifndef __PTHREAD_T_DEFINED
typedef pid_t pthread_t;
#  define __PTHREAD_T_DEFINED 1
#endif

struct pthread_condattr_s
{
  int pshared;
  clockid_t clockid;
};

#ifndef __PTHREAD_CONDATTR_T_DEFINED
typedef struct pthread_condattr_s pthread_condattr_t;
#  define __PTHREAD_CONDATTR_T_DEFINED 1
#endif

struct pthread_cond_s
{
  sem_t sem;
  clockid_t clockid;
  int wait_count;
};

#ifndef __PTHREAD_COND_T_DEFINED
typedef struct pthread_cond_s pthread_cond_t;
#  define __PTHREAD_COND_T_DEFINED 1
#endif

#define PTHREAD_COND_INITIALIZER {SEM_INITIALIZER(0), CLOCK_REALTIME }

struct pthread_mutexattr_s
{
  uint8_t pshared : 1;  /* PTHREAD_PROCESS_PRIVATE or PTHREAD_PROCESS_SHARED */
#if defined(CONFIG_PRIORITY_INHERITANCE) || defined(CONFIG_PRIORITY_PROTECT)
  uint8_t proto   : 2;  /* See PTHREAD_PRIO_* definitions */
#endif
#ifdef CONFIG_PRIORITY_PROTECT
  uint8_t ceiling;      /* Priority ceiling */
#endif
#ifdef CONFIG_PTHREAD_MUTEX_TYPES
  uint8_t type    : 2;  /* Type of the mutex.  See PTHREAD_MUTEX_* definitions */
#endif
#ifdef CONFIG_PTHREAD_MUTEX_BOTH
  uint8_t robust  : 1;  /* PTHREAD_MUTEX_STALLED or PTHREAD_MUTEX_ROBUST */
#endif
};

#ifndef __PTHREAD_MUTEXATTR_T_DEFINED
typedef struct pthread_mutexattr_s pthread_mutexattr_t;
#  define __PTHREAD_MUTEXATTR_T_DEFINED 1
#endif

struct pthread_mutex_s
{
#ifndef CONFIG_PTHREAD_MUTEX_UNSAFE
  /* Supports a singly linked list */

  FAR struct pthread_mutex_s *flink;
  uint8_t flags;    /* See _PTHREAD_MFLAGS_* */
#endif

  /* Payload */

#ifdef CONFIG_PTHREAD_MUTEX_TYPES
  uint8_t type;     /* Type of the mutex.  See PTHREAD_MUTEX_* definitions */
  rmutex_t mutex;   /* Mutex underlying the implementation of the mutex */
#else
  mutex_t mutex;    /* Mutex underlying the implementation of the mutex */
#endif
};

#ifndef __PTHREAD_MUTEX_T_DEFINED
typedef struct pthread_mutex_s pthread_mutex_t;
#  define __PTHREAD_MUTEX_T_DEFINED 1
#endif

#ifndef CONFIG_PTHREAD_MUTEX_UNSAFE
#  ifdef CONFIG_PTHREAD_MUTEX_DEFAULT_UNSAFE
#    define __PTHREAD_MUTEX_DEFAULT_FLAGS 0
#  else
#    define __PTHREAD_MUTEX_DEFAULT_FLAGS _PTHREAD_MFLAGS_ROBUST
#  endif
#endif

#ifdef CONFIG_PTHREAD_MUTEX_DEFAULT_PRIO_INHERIT
#  define PTHREAD_MUTEX_DEFAULT_PRIO_INHERIT SEM_PRIO_INHERIT
#else
#  define PTHREAD_MUTEX_DEFAULT_PRIO_INHERIT 0
#endif

#ifdef CONFIG_PTHREAD_MUTEX_DEFAULT_PRIO_PROTECT
#  define PTHREAD_MUTEX_DEFAULT_PRIO_PROTECT SEM_PRIO_PROTECT
#else
#  define PTHREAD_MUTEX_DEFAULT_PRIO_PROTECT 0
#endif

#define PTHREAD_MUTEX_DEFAULT_PRIO_FLAGS (PTHREAD_MUTEX_DEFAULT_PRIO_INHERIT | \
                                          PTHREAD_MUTEX_DEFAULT_PRIO_PROTECT)

#define PTHREAD_NXMUTEX_INITIALIZER  {                                    \
    NXSEM_INITIALIZER(NXSEM_NO_MHOLDER,                                   \
                      SEM_TYPE_MUTEX | PTHREAD_MUTEX_DEFAULT_PRIO_FLAGS)}
#define PTHREAD_NXRMUTEX_INITIALIZER {PTHREAD_NXMUTEX_INITIALIZER, 0}

#if defined(CONFIG_PTHREAD_MUTEX_TYPES) && !defined(CONFIG_PTHREAD_MUTEX_UNSAFE)
#  define PTHREAD_MUTEX_INITIALIZER {NULL, __PTHREAD_MUTEX_DEFAULT_FLAGS, \
                                     PTHREAD_MUTEX_DEFAULT, \
                                     PTHREAD_NXRMUTEX_INITIALIZER}

#  define PTHREAD_RECURSIVE_MUTEX_INITIALIZER_NP \
                                    {NULL, __PTHREAD_MUTEX_DEFAULT_FLAGS, \
                                     PTHREAD_MUTEX_RECURSIVE, \
                                     PTHREAD_NXRMUTEX_INITIALIZER,}
#elif defined(CONFIG_PTHREAD_MUTEX_TYPES)
#  define PTHREAD_MUTEX_INITIALIZER {PTHREAD_MUTEX_DEFAULT, \
                                     PTHREAD_NXRMUTEX_INITIALIZER,}
#  define PTHREAD_RECURSIVE_MUTEX_INITIALIZER_NP \
                                    {PTHREAD_MUTEX_RECURSIVE, \
                                     PTHREAD_NXRMUTEX_INITIALIZER}
#elif !defined(CONFIG_PTHREAD_MUTEX_UNSAFE)
#  define PTHREAD_MUTEX_INITIALIZER {NULL, __PTHREAD_MUTEX_DEFAULT_FLAGS,\
                                     PTHREAD_NXMUTEX_INITIALIZER}
#else
#  define PTHREAD_MUTEX_INITIALIZER {PTHREAD_NXMUTEX_INITIALIZER}
#endif

struct pthread_barrierattr_s
{
  int pshared;
};

#ifndef __PTHREAD_BARRIERATTR_T_DEFINED
typedef struct pthread_barrierattr_s pthread_barrierattr_t;
#  define __PTHREAD_BARRIERATTR_T_DEFINED 1
#endif

struct pthread_barrier_s
{
  sem_t        sem;
  unsigned int count;
  unsigned int wait_count;
  mutex_t      mutex;
};

#ifndef __PTHREAD_BARRIER_T_DEFINED
typedef struct pthread_barrier_s pthread_barrier_t;
#  define __PTHREAD_BARRIER_T_DEFINED 1
#endif

struct pthread_once_s
{
  bool done;
  pthread_mutex_t mutex;
};

#ifndef __PTHREAD_ONCE_T_DEFINED
typedef struct pthread_once_s pthread_once_t;
#  define __PTHREAD_ONCE_T_DEFINED 1
#endif

struct pthread_rwlockattr_s
{
  int pshared;
};

#ifndef __PTHREAD_RWLOCKATTR_T_DEFINED
typedef struct pthread_rwlockattr_s pthread_rwlockattr_t;
#  define __PTHREAD_RWLOCKATTR_T_DEFINED 1
#endif

struct pthread_rwlock_s
{
  pthread_mutex_t lock;
  pthread_cond_t  cv;
  unsigned int num_readers;
  unsigned int num_writers;
  bool write_in_progress;
};

#ifndef __PTHREAD_RWLOCK_T_DEFINED
typedef struct pthread_rwlock_s pthread_rwlock_t;
#  define __PTHREAD_RWLOCK_T_DEFINED 1
#endif

#define PTHREAD_RWLOCK_INITIALIZER  {PTHREAD_MUTEX_INITIALIZER, \
                                     PTHREAD_COND_INITIALIZER, \
                                     0, 0, false}

#ifdef CONFIG_PTHREAD_SPINLOCKS
/* This (non-standard) structure represents a pthread spinlock */

struct pthread_spinlock_s
{
  volatile spinlock_t sp_lock;  /* Indicates if the spinlock is locked or
                                 * not.  See the values SP_LOCKED and
                                 * SP_UNLOCKED. */
  pthread_t sp_holder;          /* ID of the thread that holds the spinlock */
};
#  ifndef __PTHREAD_SPINLOCK_T_DEFINED
/* It is referenced via this standard type */

typedef FAR struct pthread_spinlock_s pthread_spinlock_t;
#    define __PTHREAD_SPINLOCK_T_DEFINED 1
#  endif
#endif /* CONFIG_PTHREAD_SPINLOCKS */

/* This type describes the pthread cleanup callback (non-standard) */

typedef CODE void (*pthread_cleanup_t)(FAR void *arg);

/* Forward references */

struct sched_param; /* Defined in sched.h */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Initializes a thread attributes object (attr) with default values
 * for all of the individual attributes used by a given implementation.
 */

int pthread_attr_init(FAR pthread_attr_t *attr);

/* An attributes object can be deleted when it is no longer needed. */

int pthread_attr_destroy(FAR pthread_attr_t *attr);

/* Set or obtain the default scheduling algorithm */

int pthread_attr_setschedpolicy(FAR pthread_attr_t *attr, int policy);
int pthread_attr_getschedpolicy(FAR const pthread_attr_t *attr,
                                FAR int *policy);
int pthread_attr_setschedparam(FAR pthread_attr_t *attr,
                               FAR const struct sched_param *param);
int pthread_attr_getschedparam(FAR const pthread_attr_t *attr,
                               FAR struct sched_param *param);
int pthread_attr_setinheritsched(FAR pthread_attr_t *attr,
                                 int inheritsched);
int pthread_attr_getinheritsched(FAR const pthread_attr_t *attr,
                                 FAR int *inheritsched);
int pthread_attr_getdetachstate(FAR const pthread_attr_t *attr,
                                FAR int *detachstate);
int pthread_attr_setdetachstate(FAR pthread_attr_t *attr,
                                int detachstate);

#ifdef CONFIG_SMP
/* Set or obtain thread affinity attributes */

int pthread_attr_setaffinity_np(FAR pthread_attr_t *attr,
                                size_t cpusetsize,
                                FAR const cpu_set_t *cpuset);
int pthread_attr_getaffinity_np(FAR const pthread_attr_t *attr,
                                size_t cpusetsize, cpu_set_t *cpuset);
#endif

/* Set or obtain the default stack size */

int pthread_attr_setstackaddr(FAR pthread_attr_t *attr, FAR void *stackaddr);
int pthread_attr_getstackaddr(FAR const pthread_attr_t *attr,
                              FAR void **stackaddr);

/* Set or obtain the default stack size */

int pthread_attr_setstacksize(FAR pthread_attr_t *attr, size_t stacksize);
int pthread_attr_getstacksize(FAR const pthread_attr_t *attr,
                              FAR size_t *stacksize);

/* Set or obtain stack address and size attributes */

int pthread_attr_setstack(FAR pthread_attr_t *attr,
                          FAR void *stackaddr, size_t stacksize);
int pthread_attr_getstack(FAR const pthread_attr_t *attr,
                          FAR void **stackaddr, FAR size_t *stacksize);

/* Set/get contention scope attribute in thread attributes object */

int pthread_attr_setscope(FAR pthread_attr_t *attr, int scope);
int pthread_attr_getscope(FAR const pthread_attr_t *attr, FAR int *scope);

/* Set or get the name of a thread */

int pthread_setname_np(pthread_t thread, FAR const char *name);
int pthread_getname_np(pthread_t thread, FAR char *name, size_t len);

/* Get run-time stack address and size */

FAR void *pthread_get_stackaddr_np(pthread_t thread);
ssize_t pthread_get_stacksize_np(pthread_t thread);

/* To create a thread object and runnable thread, a routine must be specified
 * as the new thread's start routine.  An argument may be passed to this
 * routine, as an untyped address; an untyped address may also be returned as
 * the routine's value.  An attributes object may be used to specify details
 * about the kind of thread being created.
 */

int pthread_create(FAR pthread_t *thread, FAR const pthread_attr_t *attr,
                   pthread_startroutine_t startroutine, pthread_addr_t arg);

/* A thread object may be "detached" to specify that the return value and
 * completion status will not be requested.
 */

int pthread_detach(pthread_t thread);

/* A thread may terminate it's own execution or the execution of another
 * thread.
 */

void pthread_exit(pthread_addr_t value) noreturn_function;
int  pthread_cancel(pthread_t thread);
int  pthread_setcancelstate(int state, FAR int *oldstate);
int  pthread_setcanceltype(int type, FAR int *oldtype);
void pthread_testcancel(void);

/* A thread may set up cleanup functions to execute when the thread exits or
 * is canceled.
 */

#if CONFIG_TLS_NCLEANUP > 0
void pthread_cleanup_pop(int execute);
void pthread_cleanup_push(pthread_cleanup_t routine, FAR void *arg);
#else
#  define pthread_cleanup_pop(execute) ((void)(execute))
#  define pthread_cleanup_push(routine,arg) ((void)(routine), (void)(arg))
#endif

/* A thread can await termination of another thread and retrieve the return
 * value of the thread.
 */

int pthread_join(pthread_t thread, FAR pthread_addr_t *value);

/* A thread may tell the scheduler that its processor
 * can be made available.
 */

void pthread_yield(void);

/* A thread may obtain a copy of its own thread handle. */

pthread_t pthread_self(void);
pid_t pthread_gettid_np(pthread_t thread);

/* Compare two thread IDs. */

#define pthread_equal(t1,t2) ((t1) == (t2))

/* Thread scheduling parameters */

int pthread_getschedparam(pthread_t thread, FAR int *policy,
                          FAR struct sched_param *param);
int pthread_setschedparam(pthread_t thread, int policy,
                          FAR const struct sched_param *param);
int pthread_setschedprio(pthread_t thread, int prio);

#ifdef CONFIG_SMP
/* Thread affinity */

int pthread_setaffinity_np(pthread_t thread, size_t cpusetsize,
                           FAR const cpu_set_t *cpuset);
int pthread_getaffinity_np(pthread_t thread, size_t cpusetsize,
                           FAR cpu_set_t *cpuset);
#else
#define pthread_setaffinity_np(...) (-ENOSYS)
#define pthread_getaffinity_np(...) (-ENOSYS)
#endif

/* Thread-specific Data Interfaces */

int pthread_key_create(FAR pthread_key_t *key,
                       CODE void (*destructor)(FAR void *));
int pthread_setspecific(pthread_key_t key, FAR const void *value);
FAR void *pthread_getspecific(pthread_key_t key);
int pthread_key_delete(pthread_key_t key);

/* Create, operate on, and destroy mutex attributes. */

int pthread_mutexattr_init(FAR pthread_mutexattr_t *attr);
int pthread_mutexattr_destroy(FAR pthread_mutexattr_t *attr);
int pthread_mutexattr_getpshared(FAR const pthread_mutexattr_t *attr,
                                 FAR int *pshared);
int pthread_mutexattr_setpshared(FAR pthread_mutexattr_t *attr,
                                 int pshared);
int pthread_mutexattr_gettype(FAR const pthread_mutexattr_t *attr,
                              FAR int *type);
int pthread_mutexattr_settype(FAR pthread_mutexattr_t *attr, int type);
int pthread_mutexattr_getprotocol(FAR const pthread_mutexattr_t *attr,
                                  FAR int *protocol);
int pthread_mutexattr_setprotocol(FAR pthread_mutexattr_t *attr,
                                  int protocol);
int pthread_mutexattr_getrobust(FAR const pthread_mutexattr_t *attr,
                                FAR int *robust);
int pthread_mutexattr_setrobust(FAR pthread_mutexattr_t *attr,
                                int robust);
int pthread_mutexattr_getprioceiling(FAR const pthread_mutexattr_t *attr,
                                     FAR int *prioceiling);
int pthread_mutexattr_setprioceiling(FAR pthread_mutexattr_t *attr,
                                     int prioceiling);
int pthread_mutex_getprioceiling(FAR const pthread_mutex_t *mutex,
                                 FAR int *prioceiling);
int pthread_mutex_setprioceiling(FAR pthread_mutex_t *mutex,
                                 int prioceiling, FAR int *old_ceiling);

/* The following routines create, delete, lock and unlock mutexes. */

int pthread_mutex_init(FAR pthread_mutex_t *mutex,
                       FAR const pthread_mutexattr_t *attr);
int pthread_mutex_destroy(FAR pthread_mutex_t *mutex);
int pthread_mutex_lock(FAR pthread_mutex_t *mutex);
int pthread_mutex_timedlock(FAR pthread_mutex_t *mutex,
                            FAR const struct timespec *abs_timeout);
int pthread_mutex_trylock(FAR pthread_mutex_t *mutex);
int pthread_mutex_unlock(FAR pthread_mutex_t *mutex);

#ifndef CONFIG_PTHREAD_MUTEX_UNSAFE
/* Make sure that the pthread mutex is in a consistent state */

int pthread_mutex_consistent(FAR pthread_mutex_t *mutex);
#endif

/* Operations on condition variables */

int pthread_condattr_init(FAR pthread_condattr_t *attr);
int pthread_condattr_destroy(FAR pthread_condattr_t *attr);
int pthread_condattr_getpshared(FAR const pthread_condattr_t *attr,
                                FAR int *pshared);
int pthread_condattr_setpshared(FAR pthread_condattr_t *attr, int pshared);
int pthread_condattr_getclock(FAR const pthread_condattr_t *attr,
                              clockid_t *clock_id);
int pthread_condattr_setclock(FAR pthread_condattr_t *attr,
                              clockid_t clock_id);

/* A thread can create and delete condition variables. */

int pthread_cond_init(FAR pthread_cond_t *cond,
                      FAR const pthread_condattr_t *attr);
int pthread_cond_destroy(FAR pthread_cond_t *cond);

/* A thread can signal to and broadcast on a condition variable. */

int pthread_cond_broadcast(FAR pthread_cond_t *cond);
int pthread_cond_signal(FAR pthread_cond_t *cond);

/* A thread can wait for a condition variable to be signalled or broadcast. */

int pthread_cond_wait(FAR pthread_cond_t *cond, FAR pthread_mutex_t *mutex);

/* A thread can perform a timed wait on a condition variable. */

int pthread_cond_timedwait(FAR pthread_cond_t *cond,
                           FAR pthread_mutex_t *mutex,
                           FAR const struct timespec *abstime);

int pthread_cond_clockwait(FAR pthread_cond_t *cond,
                           FAR pthread_mutex_t *mutex,
                           clockid_t clockid,
                           FAR const struct timespec *abstime);

/* Barrier attributes */

int pthread_barrierattr_destroy(FAR pthread_barrierattr_t *attr);
int pthread_barrierattr_init(FAR pthread_barrierattr_t *attr);
int pthread_barrierattr_getpshared(FAR const pthread_barrierattr_t *attr,
                                   FAR int *pshared);
int pthread_barrierattr_setpshared(FAR pthread_barrierattr_t *attr,
                                   int pshared);

/* Barriers */

int pthread_barrier_destroy(FAR pthread_barrier_t *barrier);
int pthread_barrier_init(FAR pthread_barrier_t *barrier,
                         FAR const pthread_barrierattr_t *attr,
                         unsigned int count);
int pthread_barrier_wait(FAR pthread_barrier_t *barrier);

/* Pthread initialization */

int pthread_once(FAR pthread_once_t *once_control,
                 CODE void (*init_routine)(void));

/* Pthread rwlock attributes */

int pthread_rwlockattr_init(FAR pthread_rwlockattr_t *attr);
int pthread_rwlockattr_destroy(FAR pthread_rwlockattr_t *attr);
int pthread_rwlockattr_getpshared(FAR const pthread_rwlockattr_t *attr,
                                  FAR int *pshared);
int pthread_rwlockattr_setpshared(FAR pthread_rwlockattr_t *attr,
                                  int pshared);

/* Pthread rwlock */

int pthread_rwlock_destroy(FAR pthread_rwlock_t *rw_lock);
int pthread_rwlock_init(FAR pthread_rwlock_t *rw_lock,
                        FAR const pthread_rwlockattr_t *attr);
int pthread_rwlock_rdlock(pthread_rwlock_t *lock);
int pthread_rwlock_timedrdlock(FAR pthread_rwlock_t *lock,
                               FAR const struct timespec *abstime);
int pthread_rwlock_clockrdlock(FAR pthread_rwlock_t *lock,
                               clockid_t clockid,
                               FAR const struct timespec *abstime);
int pthread_rwlock_tryrdlock(FAR pthread_rwlock_t *lock);
int pthread_rwlock_wrlock(FAR pthread_rwlock_t *lock);
int pthread_rwlock_timedwrlock(FAR pthread_rwlock_t *lock,
                               FAR const struct timespec *abstime);
int pthread_rwlock_clockwrlock(FAR pthread_rwlock_t *lock,
                               clockid_t clockid,
                               FAR const struct timespec *abstime);
int pthread_rwlock_trywrlock(FAR pthread_rwlock_t *lock);
int pthread_rwlock_unlock(FAR pthread_rwlock_t *lock);

/* Pthread signal management APIs */

int pthread_kill(pthread_t thread, int sig);
int pthread_sigmask(int how, FAR const sigset_t *set, FAR sigset_t *oset);

#ifdef CONFIG_PTHREAD_SPINLOCKS
/* Pthread spinlocks */

int pthread_spin_init(FAR pthread_spinlock_t *lock, int pshared);
int pthread_spin_destroy(FAR pthread_spinlock_t *lock);
int pthread_spin_lock(FAR pthread_spinlock_t *lock);
int pthread_spin_trylock(FAR pthread_spinlock_t *lock);
int pthread_spin_unlock(FAR pthread_spinlock_t *lock);
#endif

int pthread_getcpuclockid(pthread_t thread_id, FAR clockid_t *clock_id);

int pthread_atfork(CODE void (*prepare)(void),
                   CODE void (*parent)(void),
                   CODE void (*child)(void));

#ifdef __cplusplus
}
#endif

/****************************************************************************
 * Minimal Type Definitions
 ****************************************************************************/

#else /* __INCLUDE_PTHREAD_H */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <stdbool.h>

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* Avoid circular dependencies by assuring that simple type definitions are
 * available in any inclusion ordering.
 */

#ifndef __PTHREAD_KEY_T_DEFINED
typedef int pthread_key_t;
#  define __PTHREAD_KEY_T_DEFINED 1
#endif

#ifndef __PTHREAD_ADDR_T_DEFINED
typedef FAR void *pthread_addr_t;
#  define __PTHREAD_ADDR_T_DEFINED 1
#endif

#ifndef __PTHREAD_ATTR_T_DEFINED
struct pthread_attr_s;
typedef struct pthread_attr_s pthread_attr_t;
#  define __PTHREAD_ATTR_T_DEFINED 1
#endif

#ifndef __PTHREAD_T_DEFINED
typedef pid_t pthread_t;
#  define __PTHREAD_T_DEFINED 1
#endif

#ifndef __PTHREAD_CONDATTR_T_DEFINED
typedef struct pthread_condattr_s pthread_condattr_t;
#  define __PTHREAD_CONDATTR_T_DEFINED 1
#endif

#ifndef __PTHREAD_COND_T_DEFINED
struct pthread_cond_s;
typedef struct pthread_cond_s pthread_cond_t;
#  define __PTHREAD_COND_T_DEFINED 1
#endif

#ifndef __PTHREAD_MUTEXATTR_T_DEFINED
struct pthread_mutexattr_s;
typedef struct pthread_mutexattr_s pthread_mutexattr_t;
#  define __PTHREAD_MUTEXATTR_T_DEFINED 1
#endif

#ifndef __PTHREAD_MUTEX_T_DEFINED
struct pthread_mutex_s;
typedef struct pthread_mutex_s pthread_mutex_t;
#  define __PTHREAD_MUTEX_T_DEFINED 1
#endif

#ifndef __PTHREAD_BARRIERATTR_T_DEFINED
struct pthread_barrierattr_s;
typedef struct pthread_barrierattr_s pthread_barrierattr_t;
#  define __PTHREAD_BARRIERATTR_T_DEFINED 1
#endif

#ifndef __PTHREAD_BARRIER_T_DEFINED
struct pthread_barrier_s;
typedef struct pthread_barrier_s pthread_barrier_t;
#  define __PTHREAD_BARRIER_T_DEFINED 1
#endif

#ifndef __PTHREAD_RWLOCKATTR_T_DEFINED
struct pthread_rwlockattr_s;
typedef struct pthread_rwlockattr_s pthread_rwlockattr_t;
#  define __PTHREAD_RWLOCKATTR_T_DEFINED 1
#endif

#ifndef __PTHREAD_RWLOCK_T_DEFINED
struct pthread_rwlock_s;
typedef struct pthread_rwlock_s pthread_rwlock_t;
#  define __PTHREAD_RWLOCK_T_DEFINED 1
#endif

#ifdef CONFIG_PTHREAD_SPINLOCKS
#  ifndef __PTHREAD_SPINLOCK_T_DEFINED
struct pthread_spinlock_s;
typedef FAR struct pthread_spinlock_s pthread_spinlock_t;
#    define __PTHREAD_SPINLOCK_T_DEFINED 1
#  endif
#endif /* CONFIG_PTHREAD_SPINLOCKS */

#ifndef __PTHREAD_ONCE_T_DEFINED
struct pthread_once_s;
typedef struct pthread_once_s pthread_once_t;
#  define __PTHREAD_ONCE_T_DEFINED 1
#endif

#endif /* __INCLUDE_PTHREAD_H */
