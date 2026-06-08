#ifndef __PROJECTS_PROJECT2_VEHICLE_H__
#define __PROJECTS_PROJECT2_VEHICLE_H__

#include "projects/crossroads/position.h"

struct lock;

#define VEHICLE_STATUS_READY 	0
#define VEHICLE_STATUS_RUNNING	1
#define VEHICLE_STATUS_FINISHED	2

#define VEHICL_TYPE_NORMAL 0
#define VEHICL_TYPE_AMBULANCE 1

struct vehicle_info {
	char id;
	char state;
	char start;
	char dest;
	
	int type;
	int arrival;
	int golden_time;
	
	struct position position;
	int route_index;
	int wait_steps;
	int input_order;
	struct lock **map_locks;
};

void vehicle_loop(void *vi);

void parse_vehicles(struct vehicle_info *vehicle_info, char *input);
void init_on_mainthread(int thread_cnt);
void init_blinker(struct lock **map_locks, struct vehicle_info *vehicle_info);
void start_blinker(void);

#endif /* __PROJECTS_PROJECT2_VEHICLE_H__ */
