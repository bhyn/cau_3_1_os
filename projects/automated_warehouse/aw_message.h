#ifndef _PROJECTS_PROJECT1_AW_MESSAGE_H__
#define _PROJECTS_PROJECT1_AW_MESSAGE_H__

/**
 * For easy to implement, combine robot and central control node message
 * If you want to modify message structure, don't split it
 */
struct message {
    //
    // To central control node
    //
    /** current row of robot */
    int row;
    /** current column of robot */
    int col;
    /** current payload of robot */
    int current_payload;
    /** required paylod of robot */
    int required_payload;

    //
    // To robots
    //
    /** next command for robot */
    int cmd;
};

/** 
 * Simple message box which can receive only one message from sender
*/
struct message_box {
    /** check if the message was written by others */
    int dirtyBit;
    /** stored message */
    struct message msg;
};

/** message boxes from central control node to each robot */
extern struct message_box* boxes_from_central_control_node;
/** message boxes from robots to central control node */
extern struct message_box* boxes_from_robots;

// 메시지 시스템 초기화 및 통신 함수
void init_message_boxes(int num_robots);
void send_message_to_central(int robot_id, struct message *msg);
struct message receive_message_from_robot(int robot_id);
void send_command_to_robot(int robot_id, int cmd);
int receive_command_from_central(int robot_id);
int is_message_from_robot_ready(int robot_id);
void clear_message_box(int robot_id, int from_central);

#endif