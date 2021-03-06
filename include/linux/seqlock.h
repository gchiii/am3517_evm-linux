#ifndef __LINUX_SEQLOCK_H
#define __LINUX_SEQLOCK_H
/*
 * Reader/writer consistent mechanism without starving writers. This type of
 * lock for data where the reader wants a consistent set of information
 * and is willing to retry if the information changes. Readers block
 * on write contention (and where applicable, pi-boost the writer).
 * Readers without contention on entry acquire the critical section
 * without any atomic operations, but they may have to retry if a writer
 * enters before the critical section ends. Writers do not wait for readers.
 *
 * This is not as cache friendly as brlock. Also, this will not work
 * for data that contains pointers, because any writer could
 * invalidate a pointer that a reader was following.
 *
 * Expected reader usage:
 * 	do {
 *	    seq = read_seqbegin(&foo);
 * 	...
 *      } while (read_seqretry(&foo, seq));
 *
 *
 * On non-SMP the spin locks disappear but the writer still needs
 * to increment the sequence variables because an interrupt routine could
 * change the state of the data.
 *
 * Based on x86_64 vsyscall gettimeofday 
 * by Keith Owens and Andrea Arcangeli
 *
 * Priority inheritance and live-lock avoidance by Gregory Haskins
 */

#include <linux/spinlock.h>
#include <linux/preempt.h>

typedef struct {
	unsigned sequence;
	raw_spinlock_t lock;
} raw_seqlock_t;

typedef struct {
	unsigned sequence;
	rwlock_t lock;
} seqlock_t;

/*
 * These macros triggered gcc-3.x compile-time problems.  We think these are
 * OK now.  Be cautious.
 */
#define __RAW_SEQLOCK_UNLOCKED(lockname) \
	{ 0, __RAW_SPIN_LOCK_UNLOCKED(lockname) }

#define raw_seqlock_init(x)				\
	do {						\
		(x)->sequence = 0;			\
		raw_spin_lock_init(&(x)->lock);	\
	} while (0)

#define DEFINE_RAW_SEQLOCK(x) \
	raw_seqlock_t x = __RAW_SEQLOCK_UNLOCKED(x)

#define __SEQLOCK_UNLOCKED(lockname) \
	{ 0, __RW_LOCK_UNLOCKED(lockname) }

#define SEQLOCK_UNLOCKED \
	__SEQLOCK_UNLOCKED(old_style_seqlock_init)

#define seqlock_init(x)					\
	do {						\
		(x)->sequence = 0;			\
		rwlock_init(&(x)->lock);		\
	} while (0)

#define DEFINE_SEQLOCK(x) \
	seqlock_t x = __SEQLOCK_UNLOCKED(x)

/* Lock out other writers and update the count.
 * Acts like a normal spin_lock/unlock.
 * Don't need preempt_disable() because that is in the spin_lock already.
 */
static inline void write_raw_seqlock(raw_seqlock_t *sl)
{
	raw_spin_lock(&sl->lock);
	++sl->sequence;
	smp_wmb();
}

static inline void write_seqlock(seqlock_t *sl)
{
	write_lock(&sl->lock);
	++sl->sequence;
	smp_wmb();
}

static inline void write_raw_sequnlock(raw_seqlock_t *sl)
{
	smp_wmb();
	sl->sequence++;
	raw_spin_unlock(&sl->lock);
}

static inline void write_sequnlock(seqlock_t *sl)
{
	smp_wmb();
	sl->sequence++;
	write_unlock(&sl->lock);
}

static inline int write_tryseqlock(seqlock_t *sl)
{
	int ret = write_trylock(&sl->lock);

	if (ret) {
		++sl->sequence;
		smp_wmb();
	}
	return ret;
}

/* Start of read calculation -- fetch last complete writer token */
static __always_inline unsigned read_raw_seqbegin(const raw_seqlock_t *sl)
{
	unsigned ret;

repeat:
	ret = ACCESS_ONCE(sl->sequence);
	if (unlikely(ret & 1)) {
		cpu_relax();
		goto repeat;
	}
	smp_rmb();

	return ret;
}

static __always_inline unsigned read_seqbegin(seqlock_t *sl)
{
	unsigned ret;

	ret = sl->sequence;
	smp_rmb();
	if (unlikely(ret & 1)) {
		cpu_relax();
		/*
		 * Serialze with the writer which will ensure they are
		 * pi-boosted if necessary and prevent us from starving
		 * them.
		 */
		read_lock(&sl->lock);
		ret = sl->sequence;
		read_unlock(&sl->lock);
	}

	BUG_ON(ret & 1);

	return ret;
}

/*
 * Test if reader processed invalid data.
 *
 * If sequence value changed then writer changed data while in section.
 */
static __always_inline int
read_raw_seqretry(const raw_seqlock_t *sl, unsigned start)
{
	smp_rmb();

	return (sl->sequence != start);
}

static __always_inline int read_seqretry(const seqlock_t *sl, unsigned start)
{
	smp_rmb();

	return (sl->sequence != start);
}


/*
 * Version using sequence counter only.
 * This can be used when code has its own mutex protecting the
 * updating starting before the write_seqcountbeqin() and ending
 * after the write_seqcount_end().
 */

typedef struct seqcount {
	unsigned sequence;
} seqcount_t;

#define SEQCNT_ZERO { 0 }
#define seqcount_init(x)	do { *(x) = (seqcount_t) SEQCNT_ZERO; } while (0)

/* Start of read using pointer to a sequence counter only.  */
static inline unsigned read_seqcount_begin(const seqcount_t *s)
{
	unsigned ret;

repeat:
	ret = s->sequence;
	smp_rmb();
	if (unlikely(ret & 1)) {
		cpu_relax();
		goto repeat;
	}
	return ret;
}

/*
 * Test if reader processed invalid data because sequence number has changed.
 */
static inline int read_seqcount_retry(const seqcount_t *s, unsigned start)
{
	smp_rmb();

	return s->sequence != start;
}


/*
 * Sequence counter only version assumes that callers are using their
 * own mutexing.
 */
static inline void write_seqcount_begin(seqcount_t *s)
{
	s->sequence++;
	smp_wmb();
}

static inline void write_seqcount_end(seqcount_t *s)
{
	smp_wmb();
	s->sequence++;
}

/*
 * Possible sw/hw IRQ protected versions of the interfaces.
 */
#define write_raw_seqlock_irqsave(lock, flags)				\
	do { local_irq_save(flags); write_raw_seqlock(lock); } while (0)
#define write_raw_seqlock_irq(lock)					\
	do { local_irq_disable();   write_raw_seqlock(lock); } while (0)
#define write_raw_seqlock_bh(lock)					\
	do { local_bh_disable();    write_raw_seqlock(lock); } while (0)

#define write_raw_sequnlock_irqrestore(lock, flags)			\
	do { write_raw_sequnlock(lock); local_irq_restore(flags); } while(0)
#define write_raw_sequnlock_irq(lock)					\
	do { write_raw_sequnlock(lock); local_irq_enable(); } while(0)
#define write_raw_sequnlock_bh(lock)					\
	do { write_raw_sequnlock(lock); local_bh_enable(); } while(0)

#define read_raw_seqbegin_irqsave(lock, flags)				\
	({ local_irq_save(flags);   read_raw_seqbegin(lock); })

#define read_raw_seqretry_irqrestore(lock, iv, flags)			\
	({								\
		int ret = read_raw_seqretry(lock, iv);			\
		local_irq_restore(flags);				\
		ret;							\
	})

#define write_seqlock_irqsave(lock, flags)				\
	do { local_irq_save(flags); write_seqlock(lock); } while (0)
#define write_seqlock_irq(lock)						\
	do { local_irq_disable();   write_seqlock(lock); } while (0)
#define write_seqlock_bh(lock)						\
	do { local_bh_disable();    write_seqlock(lock); } while (0)

#define write_sequnlock_irqrestore(lock, flags)				\
	do { write_sequnlock(lock); local_irq_restore(flags); } while(0)
#define write_sequnlock_irq(lock)					\
	do { write_sequnlock(lock); local_irq_enable(); } while(0)
#define write_sequnlock_bh(lock)					\
	do { write_sequnlock(lock); local_bh_enable(); } while(0)

#define read_seqbegin_irqsave(lock, flags)				\
	({ local_irq_save(flags);   read_seqbegin(lock); })

#define read_seqretry_irqrestore(lock, iv, flags)			\
	({								\
		int ret = read_seqretry(lock, iv);			\
		local_irq_restore(flags);				\
		ret;							\
	})

#endif /* __LINUX_SEQLOCK_H */
