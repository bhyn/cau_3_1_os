#include "projects/automated_warehouse/aw_message.h"
#include "threads/malloc.h"
#include <stdio.h>

// 전역 메시지 박스
struct message_box* boxes_from_central_control_node = NULL;
struct message_box* boxes_from_robots = NULL;

/**
 * 메시지 박스 시스템 초기화
 * 파라미터: num_robots - 로봇 개수
 * 
 * 역할: 각 로봇마다 메시지 박스 2개씩 할당
 *       - boxes_from_robots[i]: i번 로봇이 중앙으로 보내는 메시지
 *       - boxes_from_central_control_node[i]: 중앙이 i번 로봇으로 보내는 메시지
 */
void init_message_boxes(int num_robots) {
    // 로봇별 메시지 박스 배열 동적 할당
    boxes_from_robots = malloc(sizeof(struct message_box) * num_robots);
    boxes_from_central_control_node = malloc(sizeof(struct message_box) * num_robots);
    
    if (boxes_from_robots == NULL || boxes_from_central_control_node == NULL) {
        printf("[ERROR] Failed to allocate message boxes\n");
        return;
    }
    
    // 모든 메시지 박스 초기화
    for (int i = 0; i < num_robots; i++) {
        boxes_from_robots[i].dirtyBit = 0;
        boxes_from_central_control_node[i].dirtyBit = 0;
    }
    
    printf("[INFO] Message boxes initialized for %d robots\n", num_robots);
}

/**
 * 로봇이 중앙 관제에 메시지 전송
 * 파라미터: robot_id - 로봇 인덱스
 *          msg - 전송할 메시지
 */
void send_message_to_central(int robot_id, struct message *msg) {
    boxes_from_robots[robot_id].msg = *msg;
    boxes_from_robots[robot_id].dirtyBit = 1;  // 새 메시지 도착 표시
    
    printf("[MSG] Robot %d sent: row=%d, col=%d, payload=%d\n",
           robot_id, msg->row, msg->col, msg->current_payload);
}

/**
 * 중앙 관제가 로봇으로부터 메시지 수신
 * 파라미터: robot_id - 로봇 인덱스
 * 반환: 수신한 메시지
 */
struct message receive_message_from_robot(int robot_id) {
    return boxes_from_robots[robot_id].msg;
}

/**
 * 중앙이 로봇에게 명령 전송
 * 파라미터: robot_id - 로봇 인덱스
 *          cmd - 전송할 명령
 */
void send_command_to_robot(int robot_id, int cmd) {
    boxes_from_central_control_node[robot_id].msg.cmd = cmd;
    boxes_from_central_control_node[robot_id].dirtyBit = 1;
    
    printf("[CMD] Sent command %d to robot %d\n", cmd, robot_id);
}

/**
 * 로봇이 중앙으로부터 명령 수신
 * 파라미터: robot_id - 로봇 인덱스
 * 반환: 수신한 명령
 */
int receive_command_from_central(int robot_id) {
    return boxes_from_central_control_node[robot_id].msg.cmd;
}

/**
 * 로봇으로부터 새로운 메시지가 도착했는지 확인
 * 파라미터: robot_id - 로봇 인덱스
 * 반환: 1이면 새 메시지 있음, 0이면 없음
 */
int is_message_from_robot_ready(int robot_id) {
    return boxes_from_robots[robot_id].dirtyBit;
}

/**
 * 메시지 박스 클리어 (메시지를 읽음으로 표시)
 * 파라미터: robot_id - 로봇 인덱스
 *          from_central - 1이면 중앙->로봇, 0이면 로봇->중앙
 */
void clear_message_box(int robot_id, int from_central) {
    if (from_central) {
        boxes_from_central_control_node[robot_id].dirtyBit = 0;
    } else {
        boxes_from_robots[robot_id].dirtyBit = 0;
    }
}