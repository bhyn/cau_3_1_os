#ifndef _PROJECTS_PROJECT1_ROBOT_H__
#define _PROJECTS_PROJECT1_ROBOT_H__

/**
 * A Structure representing robot
 */
struct robot {
    const char* name;
    char required_item; //적재할 물건 번호
    char destination; //하역 장소
    int row;
    int col;
    int required_payload; //?
    int current_payload; //?
    int task_completed; //작업 완료 여부
};

void setRobot(struct robot* _robot, const char* name, char required_item, char destination, int row, int col, int required_payload, int current_payload, int task_completed);
void parse_robot_tasks(const char *task_string, struct robot *robots, int num_robots);

#endif
