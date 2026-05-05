#include "projects/automated_warehouse/robot.h"
#include "projects/automated_warehouse/aw_manager.h"
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
    // task_string을 수정하지 않기 위해 복사
    char task_copy[256];
    strlcpy(task_copy, task_string, sizeof(task_copy));
    
    char *saveptr;
    char *task = strtok_r(task_copy, ":", &saveptr);

    for (int i = 0; i < num_robots && task != NULL; i++) {
        int item = task[0] - '0';           
        char destination = task[1];        

        // 모든 로봇은 W(대기장소)에서 시작! 중요!!
        setRobot(&robots[i], 
                 NULL,                      // 이름은 나중에 automated_warehouse.c에서 설정
                 item,                      // 물건 번호
                 destination,               // 하역 장소
                 ROW_W, COL_W,              // W 위치 (6, 5)
                 item,                      // required_payload
                 0);                        // current_payload (처음엔 아무것도 안 들음)

        task = strtok_r(NULL, ":", &saveptr);
    }
}