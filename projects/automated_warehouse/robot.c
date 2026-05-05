#include "projects/automated_warehouse/robot.h"
#include <string.h>
#include <stdlib.h>
/**
 * A function setting up robot structure
 */
void setRobot(struct robot* _robot, const char* name, char required_item, char destination, int row, int col, int required_payload, int current_payload){
    _robot->name = name;
    _robot->required_item = required_item;
    _robot->destination = destination;
    _robot->row = row;
    _robot->col = col;
    _robot->required_payload = required_payload;
    _robot->current_payload = current_payload;
}

void parse_robot_tasks(const char *task_string, struct robot *robots, int num_robots){
    char *task = strtok(task_string, ":");

    for (int i=0; i<num_robots&&task!=NULL ;i++){
        int item = task[0] -'0';
        char destination = task[1];

        setRobot(&robots[i], NULL, item, destination, 0, 0, item, 0);

        task = strtok(NULL, ":");
    }
}