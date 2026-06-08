#ifndef __PROJECTS_PROJECT2_BLINKER_H__
#define __PROJECTS_PROJECT2_BLINKER_H__

#include "projects/crossroads/vehicle.h"

struct blinker_info {
    struct lock **map_locks;
    struct vehicle_info *vehicles;
};

void init_blinker(struct lock **map_locks, struct vehicle_info *vehicle_info);
void start_blinker(void);

#endif /* __PROJECTS_PROJECT2_BLINKER_H__ */
