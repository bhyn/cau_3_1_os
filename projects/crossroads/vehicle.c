#include <debug.h>
#include <limits.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>

#include "threads/interrupt.h"
#include "threads/malloc.h"
#include "threads/synch.h"
#include "threads/thread.h"
#include "projects/crossroads/ats.h"
#include "projects/crossroads/crossroads.h"
#include "projects/crossroads/map.h"
#include "projects/crossroads/vehicle.h"

#define MAP_SIZE 7
#define VEHICLE_PRIORITY_NORMAL_BASE 1000
#define VEHICLE_PRIORITY_AMBULANCE_BASE 100000

/* path. A:0 B:1 C:2 D:3 */
const struct position vehicle_path[4][4][12] = {
	/* from A */ {
		/* to A */
		{{4,0},{4,1},{4,2},{4,3},{4,4},{3,4},{2,4},{2,3},{2,2},{2,1},{2,0},{-1,-1},},
		/* to B */
		{{4,0},{4,1},{4,2},{5,2},{6,2},{-1,-1},},
		/* to C */
		{{4,0},{4,1},{4,2},{4,3},{4,4},{4,5},{4,6},{-1,-1},},
		/* to D */
		{{4,0},{4,1},{4,2},{4,3},{4,4},{3,4},{2,4},{1,4},{0,4},{-1,-1},}
	},
	/* from B */ {
		/* to A */
		{{6,4},{5,4},{4,4},{3,4},{2,4},{2,3},{2,2},{2,1},{2,0},{-1,-1},},
		/* to B */
		{{6,4},{5,4},{4,4},{3,4},{2,4},{2,3},{2,2},{3,2},{4,2},{5,2},{6,2},{-1,-1},},
		/* to C */
		{{6,4},{5,4},{4,4},{4,5},{4,6},{-1,-1},},
		/* to D */
		{{6,4},{5,4},{4,4},{3,4},{2,4},{1,4},{0,4},{-1,-1},}
	},
	/* from C */ {
		/* to A */
		{{2,6},{2,5},{2,4},{2,3},{2,2},{2,1},{2,0},{-1,-1},},
		/* to B */
		{{2,6},{2,5},{2,4},{2,3},{2,2},{3,2},{4,2},{5,2},{6,2},{-1,-1},},
		/* to C */
		{{2,6},{2,5},{2,4},{2,3},{2,2},{3,2},{4,2},{4,3},{4,4},{4,5},{4,6},{-1,-1},},
		/* to D */
		{{2,6},{2,5},{2,4},{1,4},{0,4},{-1,-1},}
	},
	/* from D */ {
		/* to A */
		{{0,2},{1,2},{2,2},{2,1},{2,0},{-1,-1},},
		/* to B */
		{{0,2},{1,2},{2,2},{3,2},{4,2},{5,2},{6,2},{-1,-1},},
		/* to C */
		{{0,2},{1,2},{2,2},{3,2},{4,2},{4,3},{4,4},{4,5},{4,6},{-1,-1},},
		/* to D */
		{{0,2},{1,2},{2,2},{3,2},{4,2},{4,3},{4,4},{3,4},{2,4},{1,4},{0,4},{-1,-1},}
	}
};

struct priority_waiter {
	struct list_elem elem;
	struct semaphore sema;
	int priority;
	int sequence;
};

struct priority_semaphore {
	unsigned value;
	struct list waiters;
};

struct priority_lock {
	struct thread *holder;
	struct priority_semaphore semaphore;
};

struct priority_cond_waiter {
	struct list_elem elem;
	struct priority_semaphore sema;
	int priority;
	int sequence;
};

struct priority_condition {
	struct list waiters;
};

struct move_plan {
	bool candidate;
	bool selected;
	struct position next;
	int priority;
};

static struct priority_lock scheduler_lock;
static struct priority_condition step_cond;
static struct vehicle_info *vehicles;
static struct move_plan *plans;
static int vehicle_cnt;
static int waiting_cnt;
static int finished_cnt;
static int wait_sequence;
static bool scheduler_ready;

static int
allocate_wait_sequence (void)
{
	enum intr_level old_level;
	int sequence;

	old_level = intr_disable ();
	sequence = wait_sequence++;
	intr_set_level (old_level);
	return sequence;
}

static bool
priority_waiter_less (const struct list_elem *a,
		      const struct list_elem *b,
		      void *aux UNUSED)
{
	const struct priority_waiter *wa;
	const struct priority_waiter *wb;

	wa = list_entry (a, struct priority_waiter, elem);
	wb = list_entry (b, struct priority_waiter, elem);

	if (wa->priority != wb->priority)
		return wa->priority > wb->priority;
	return wa->sequence < wb->sequence;
}

static bool
priority_cond_waiter_less (const struct list_elem *a,
			   const struct list_elem *b,
			   void *aux UNUSED)
{
	const struct priority_cond_waiter *wa;
	const struct priority_cond_waiter *wb;

	wa = list_entry (a, struct priority_cond_waiter, elem);
	wb = list_entry (b, struct priority_cond_waiter, elem);

	if (wa->priority != wb->priority)
		return wa->priority > wb->priority;
	return wa->sequence < wb->sequence;
}

static void
priority_sema_init (struct priority_semaphore *sema, unsigned value)
{
	ASSERT (sema != NULL);
	sema->value = value;
	list_init (&sema->waiters);
}

static void
priority_sema_down (struct priority_semaphore *sema, int priority)
{
	enum intr_level old_level;
	struct priority_waiter waiter;

	ASSERT (sema != NULL);
	ASSERT (!intr_context ());

	old_level = intr_disable ();
	if (sema->value > 0) {
		sema->value--;
		intr_set_level (old_level);
		return;
	}

	waiter.priority = priority;
	waiter.sequence = allocate_wait_sequence ();
	sema_init (&waiter.sema, 0);
	list_insert_ordered (&sema->waiters, &waiter.elem,
			     priority_waiter_less, NULL);
	intr_set_level (old_level);

	sema_down (&waiter.sema);
}

static void
priority_sema_up (struct priority_semaphore *sema)
{
	enum intr_level old_level;

	ASSERT (sema != NULL);

	old_level = intr_disable ();
	if (!list_empty (&sema->waiters)) {
		struct priority_waiter *waiter;

		waiter = list_entry (list_pop_front (&sema->waiters),
				     struct priority_waiter, elem);
		sema_up (&waiter->sema);
	} else {
		sema->value++;
	}
	intr_set_level (old_level);
}

static void
priority_lock_init (struct priority_lock *lock)
{
	ASSERT (lock != NULL);
	lock->holder = NULL;
	priority_sema_init (&lock->semaphore, 1);
}

static void
priority_lock_acquire (struct priority_lock *lock, int priority)
{
	ASSERT (lock != NULL);
	ASSERT (!intr_context ());
	ASSERT (lock->holder != thread_current ());

	priority_sema_down (&lock->semaphore, priority);
	lock->holder = thread_current ();
}

static void
priority_lock_release (struct priority_lock *lock)
{
	ASSERT (lock != NULL);
	ASSERT (lock->holder == thread_current ());

	lock->holder = NULL;
	priority_sema_up (&lock->semaphore);
}

static void
priority_cond_init (struct priority_condition *cond)
{
	ASSERT (cond != NULL);
	list_init (&cond->waiters);
}

static void
priority_cond_wait (struct priority_condition *cond,
		    struct priority_lock *lock,
		    int priority)
{
	struct priority_cond_waiter waiter;

	ASSERT (cond != NULL);
	ASSERT (lock != NULL);
	ASSERT (!intr_context ());
	ASSERT (lock->holder == thread_current ());

	priority_sema_init (&waiter.sema, 0);
	waiter.priority = priority;
	waiter.sequence = allocate_wait_sequence ();
	list_insert_ordered (&cond->waiters, &waiter.elem,
			     priority_cond_waiter_less, NULL);

	priority_lock_release (lock);
	priority_sema_down (&waiter.sema, priority);
	priority_lock_acquire (lock, priority);
}

static void
priority_cond_broadcast (struct priority_condition *cond,
			 struct priority_lock *lock)
{
	ASSERT (cond != NULL);
	ASSERT (lock != NULL);
	ASSERT (lock->holder == thread_current ());

	while (!list_empty (&cond->waiters)) {
		struct priority_cond_waiter *waiter;

		waiter = list_entry (list_pop_front (&cond->waiters),
				     struct priority_cond_waiter, elem);
		priority_sema_up (&waiter->sema);
	}
}

static int
count_vehicle_specs (const char *input)
{
	int count;

	if (input == NULL || input[0] == '\0')
		return 1;

	count = 1;
	while (*input != '\0') {
		if (*input == ':')
			count++;
		input++;
	}
	return count;
}

static int
parse_number (const char **cursor)
{
	int value;

	value = 0;
	while (**cursor >= '0' && **cursor <= '9') {
		value = value * 10 + (**cursor - '0');
		(*cursor)++;
	}
	return value;
}

static char
normalize_point (char point)
{
	if (point >= 'a' && point <= 'd')
		point = point - 'a' + 'A';
	if (point < 'A' || point > 'D')
		return 'A';
	return point;
}

static void
init_vehicle_info (struct vehicle_info *vi, int index)
{
	vi->id = 'a' + index;
	vi->state = VEHICLE_STATUS_READY;
	vi->start = 'A';
	vi->dest = 'A';
	vi->type = VEHICL_TYPE_NORMAL;
	vi->arrival = -1;
	vi->golden_time = -1;
	vi->position.row = -1;
	vi->position.col = -1;
	vi->route_index = -1;
	vi->wait_steps = 0;
	vi->input_order = index;
	vi->map_locks = NULL;
}

void
parse_vehicles (struct vehicle_info *vehicle_info, char *input)
{
	const char *cursor;
	int count;
	int idx;

	count = count_vehicle_specs (input);
	for (idx = 0; idx < count; idx++)
		init_vehicle_info (&vehicle_info[idx], idx);

	vehicles = vehicle_info;
	vehicle_cnt = count;

	if (input == NULL)
		return;

	cursor = input;
	idx = 0;
	while (*cursor != '\0' && idx < count) {
		struct vehicle_info *vi;
		bool has_time_limit;

		vi = &vehicle_info[idx];
		if (*cursor != ':' && *cursor != '\0')
			vi->id = *cursor++;

		if (*cursor != ':' && *cursor != '\0')
			vi->start = normalize_point (*cursor++);
		if (*cursor != ':' && *cursor != '\0')
			vi->dest = normalize_point (*cursor++);

		has_time_limit = (*cursor >= '0' && *cursor <= '9');
		if (has_time_limit) {
			vi->type = VEHICL_TYPE_AMBULANCE;
			vi->arrival = parse_number (&cursor);
			vi->golden_time = -1;
			if (*cursor == '.') {
				cursor++;
				vi->golden_time = parse_number (&cursor);
			}
		}

		while (*cursor != '\0' && *cursor != ':')
			cursor++;
		if (*cursor == ':')
			cursor++;
		idx++;
	}
}

static bool
is_position_outside (struct position pos)
{
	return pos.row == -1 || pos.col == -1;
}

static bool
is_position_inside (struct position pos)
{
	return pos.row >= 0 && pos.row < MAP_SIZE
		&& pos.col >= 0 && pos.col < MAP_SIZE;
}

static int
path_start_index (const struct vehicle_info *vi)
{
	return vi->start - 'A';
}

static int
path_dest_index (const struct vehicle_info *vi)
{
	return vi->dest - 'A';
}

static bool
vehicle_can_depart_this_step (const struct vehicle_info *vi)
{
	if (vi->type != VEHICL_TYPE_AMBULANCE)
		return true;

	return vi->arrival <= crossroads_step + 1;
}

static struct position
next_position_for (const struct vehicle_info *vi)
{
	int start;
	int dest;
	int index;

	start = path_start_index (vi);
	dest = path_dest_index (vi);
	index = (vi->state == VEHICLE_STATUS_READY) ? 0 : vi->route_index + 1;
	return vehicle_path[start][dest][index];
}

static int
runtime_priority (const struct vehicle_info *vi)
{
	if (vi->type == VEHICL_TYPE_AMBULANCE) {
		int remaining;

		if (vi->golden_time < 0)
			return PRI_MAX;
		remaining = vi->golden_time - crossroads_step;
		if (remaining < 0)
			return PRI_MAX;
		if (remaining > PRI_MAX)
			return PRI_DEFAULT + 1;
		return PRI_MAX - remaining;
	}
	return PRI_DEFAULT;
}

static bool
plan_has_higher_priority (int left, int right)
{
	const struct vehicle_info *lv;
	const struct vehicle_info *rv;

	if (right < 0)
		return true;

	if (plans[left].priority != plans[right].priority)
		return plans[left].priority > plans[right].priority;

	lv = &vehicles[left];
	rv = &vehicles[right];
	if (lv->type != rv->type)
		return lv->type == VEHICL_TYPE_AMBULANCE;
	if (lv->golden_time != rv->golden_time) {
		if (lv->golden_time < 0)
			return false;
		if (rv->golden_time < 0)
			return true;
		return lv->golden_time < rv->golden_time;
	}
	if (lv->wait_steps != rv->wait_steps)
		return lv->wait_steps > rv->wait_steps;
	return lv->input_order < rv->input_order;
}

static int
base_move_priority (const struct vehicle_info *vi)
{
	int priority;

	if (vi->type == VEHICL_TYPE_AMBULANCE) {
		int remaining;

		priority = VEHICLE_PRIORITY_AMBULANCE_BASE;
		if (vi->golden_time >= 0) {
			remaining = vi->golden_time - (crossroads_step + 1);
			if (remaining < 0)
				priority += 50000 + (-remaining * 100);
			else
				priority += (1000 - remaining) * 10;
		}
	} else {
		priority = VEHICLE_PRIORITY_NORMAL_BASE;
	}

	priority += vi->wait_steps * 5;
	priority += vehicle_cnt - vi->input_order;
	return priority;
}

static void
build_occupancy (int occupancy[MAP_SIZE][MAP_SIZE])
{
	int row;
	int col;
	int i;

	for (row = 0; row < MAP_SIZE; row++) {
		for (col = 0; col < MAP_SIZE; col++)
			occupancy[row][col] = -1;
	}

	for (i = 0; i < vehicle_cnt; i++) {
		struct position pos;

		if (vehicles[i].state != VEHICLE_STATUS_RUNNING)
			continue;

		pos = vehicles[i].position;
		if (is_position_inside (pos))
			occupancy[pos.row][pos.col] = i;
	}
}

static void
build_move_candidates (void)
{
	int i;

	for (i = 0; i < vehicle_cnt; i++) {
		struct vehicle_info *vi;

		vi = &vehicles[i];
		plans[i].candidate = false;
		plans[i].selected = false;
		plans[i].next.row = -1;
		plans[i].next.col = -1;
		plans[i].priority = INT_MIN;

		if (vi->state == VEHICLE_STATUS_FINISHED)
			continue;
		if (vi->state == VEHICLE_STATUS_READY
		    && !vehicle_can_depart_this_step (vi))
			continue;

		plans[i].candidate = true;
		plans[i].next = next_position_for (vi);
		plans[i].priority = base_move_priority (vi);
	}
}

static void
propagate_blocker_priority (int occupancy[MAP_SIZE][MAP_SIZE])
{
	int pass;

	for (pass = 0; pass < vehicle_cnt; pass++) {
		bool changed;
		int i;

		changed = false;
		for (i = 0; i < vehicle_cnt; i++) {
			struct position target;
			int blocker;

			if (!plans[i].candidate)
				continue;

			target = plans[i].next;
			if (!is_position_inside (target))
				continue;

			blocker = occupancy[target.row][target.col];
			if (blocker < 0 || blocker == i || !plans[blocker].candidate)
				continue;

			if (plans[blocker].priority < plans[i].priority + 1) {
				plans[blocker].priority = plans[i].priority + 1;
				changed = true;
			}
		}

		if (!changed)
			break;
	}
}

static void
choose_target_winners (void)
{
	int target_owner[MAP_SIZE][MAP_SIZE];
	int row;
	int col;
	int i;

	for (row = 0; row < MAP_SIZE; row++) {
		for (col = 0; col < MAP_SIZE; col++)
			target_owner[row][col] = -1;
	}

	for (i = 0; i < vehicle_cnt; i++) {
		struct position target;
		int owner;

		if (!plans[i].candidate)
			continue;

		target = plans[i].next;
		if (is_position_outside (target)) {
			plans[i].selected = true;
			continue;
		}

		owner = target_owner[target.row][target.col];
		if (plan_has_higher_priority (i, owner))
			target_owner[target.row][target.col] = i;
	}

	for (i = 0; i < vehicle_cnt; i++) {
		struct position target;

		if (!plans[i].candidate || plans[i].selected)
			continue;

		target = plans[i].next;
		if (target_owner[target.row][target.col] == i)
			plans[i].selected = true;
	}
}

static void
remove_blocked_moves (int occupancy[MAP_SIZE][MAP_SIZE])
{
	bool changed;

	do {
		int i;

		changed = false;
		for (i = 0; i < vehicle_cnt; i++) {
			struct position target;
			int blocker;

			if (!plans[i].selected)
				continue;

			target = plans[i].next;
			if (!is_position_inside (target))
				continue;

			blocker = occupancy[target.row][target.col];
			if (blocker < 0 || blocker == i)
				continue;

			if (!plans[blocker].selected) {
				plans[i].selected = false;
				changed = true;
			}
		}
	} while (changed);
}

static void
apply_move_plan (void)
{
	int i;

	for (i = 0; i < vehicle_cnt; i++) {
		struct vehicle_info *vi;

		vi = &vehicles[i];
		if (!plans[i].candidate)
			continue;

		if (!plans[i].selected) {
			vi->wait_steps++;
			continue;
		}

		vi->wait_steps = 0;
		if (is_position_outside (plans[i].next)) {
			vi->position.row = -1;
			vi->position.col = -1;
			vi->state = VEHICLE_STATUS_FINISHED;
			vi->route_index++;
			finished_cnt++;
			continue;
		}

		if (vi->state == VEHICLE_STATUS_READY) {
			vi->state = VEHICLE_STATUS_RUNNING;
			vi->route_index = 0;
		} else {
			vi->route_index++;
		}
		vi->position = plans[i].next;
	}
}

static void
run_scheduler_step (void)
{
	int occupancy[MAP_SIZE][MAP_SIZE];

	ASSERT (scheduler_lock.holder == thread_current ());

	build_occupancy (occupancy);
	build_move_candidates ();
	propagate_blocker_priority (occupancy);
	choose_target_winners ();
	remove_blocked_moves (occupancy);
	apply_move_plan ();

	crossroads_step++;
	unitstep_changed ();
}

void
init_on_mainthread (int thread_cnt)
{
	priority_lock_init (&scheduler_lock);
	priority_cond_init (&step_cond);
	waiting_cnt = 0;
	finished_cnt = 0;
	wait_sequence = 0;
	scheduler_ready = true;

	vehicle_cnt = thread_cnt;
	if (plans != NULL)
		free (plans);
	plans = malloc (sizeof *plans * thread_cnt);
	ASSERT (plans != NULL);
}

void
vehicle_loop (void *_vi)
{
	struct vehicle_info *vi;

	vi = _vi;
	ASSERT (scheduler_ready);

	thread_set_priority (runtime_priority (vi));
	priority_lock_acquire (&scheduler_lock, runtime_priority (vi));
	while (vi->state != VEHICLE_STATUS_FINISHED) {
		int observed_step;
		int active_cnt;

		observed_step = crossroads_step;
		active_cnt = vehicle_cnt - finished_cnt;
		waiting_cnt++;

		if (waiting_cnt == active_cnt) {
			waiting_cnt = 0;
			run_scheduler_step ();
			priority_cond_broadcast (&step_cond, &scheduler_lock);
		} else {
			while (observed_step == crossroads_step
			       && vi->state != VEHICLE_STATUS_FINISHED) {
				priority_cond_wait (&step_cond, &scheduler_lock,
						    runtime_priority (vi));
			}
		}

		thread_set_priority (runtime_priority (vi));
	}
	priority_lock_release (&scheduler_lock);
}
