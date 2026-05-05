#include <stdio.h>
#include <string.h>

#include "threads/init.h"
#include "threads/malloc.h"
#include "threads/synch.h"
#include "threads/thread.h"

#include "devices/timer.h"

#include "projects/automated_warehouse/aw_manager.h"
#include "projects/automated_warehouse/robot.h"
#include "projects/automated_warehouse/aw_message.h"
#include "projects/automated_warehouse/aw_thread.h"

struct robot* robots;
int number_of_robots;


// 중앙 관제 스레드 함수
void central_control_thread(void* aux) {
    int num_robots_ptr = (int)aux;  // 실제로는 void*로 num_robots 받음
    
    // 해석: aux가 NULL이면 global number_of_robots 사용
    int n_robots = number_of_robots;
    
    while(1){
        printf("\n[CENTRAL] === STEP %d START ===\n", step);
        
        // 1️⃣ 모든 로봇으로부터 상태 메시지 수집
        printf("[CENTRAL] 모든 로봇의 상태를 대기 중...\n");
        int all_ready = 0;
        int timeout = 0;
        
        while (!all_ready && timeout < 100) {
            all_ready = 1;
            for (int i = 0; i < n_robots; i++) {
                if (!is_message_from_robot_ready(i)) {
                    all_ready = 0;
                    break;
                }
            }
            if (!all_ready) {
                timer_sleep(10);  // 로봇들이 메시지 보낼 시간 주기
                timeout++;
            }
        }
        
        printf("[CENTRAL] 모든 로봇 상태 수집 완료!\n");
        
        // 2️⃣ 맵 출력
        print_map(robots, n_robots);
        
        // 3️⃣ 각 로봇에게 명령 전송 (일단 WAIT 명령만 전송)
        printf("[CENTRAL] 각 로봇에게 명령 전송...\n");
        for (int i = 0; i < n_robots; i++) {
            send_command_to_robot(i, 0);  // 0 = WAIT 명령
        }
        
        // 4️⃣ 스텝 증가
        increase_step();
        
        // 5️⃣ 모든 로봇 깨우기 (UNBLOCK)
        printf("[CENTRAL] 모든 로봇을 UNBLOCK 합니다...\n");
        unblock_threads();
        
        printf("[CENTRAL] === STEP %d END ===\n\n", step - 1);
        
        timer_sleep(1000);  // 1초 대기
    }
}
// 셀에 장애물이 있는지 확인하는 함수
static int
is_blocked_cell(int row, int col)
{
        if (row < 0 || row >= MAP_HEIGHT || col < 0 || col >= MAP_WIDTH)
                return 1;
        return map_draw_default[row][col] == 'X';
}
// 물건 위치를 반환하는 함수
static void
get_item_position(char item, int *row, int *col)
{
        for (int r = 0; r < MAP_HEIGHT; r++) {
                for (int c = 0; c < MAP_WIDTH; c++) {
                        if (map_draw_default[r][c] == item) {
                                *row = r;
                                *col = c;
                                return;
                        }
                }
        }

        *row = ROW_S;
        *col = COL_S;
}
// 목적지 위치를 반환하는 함수
static void
get_destination_position(char destination, int *row, int *col)
{
        switch (destination) {
        case 'A':
                *row = ROW_A;
                *col = COL_A;
                break;
        case 'B':
                *row = ROW_B;
                *col = COL_B;
                break;
        case 'C':
                *row = ROW_C;
                *col = COL_C;
                break;
        default:
                *row = ROW_W;
                *col = COL_W;
                break;
        }
}
// BFS를 이용하여 로봇을 목표 위치로 이동시키는 함수
static void
move_robot_to(struct robot *robot, int target_row, int target_col)
{
        // ... existing code ...
}

// 한 칸씩 이동하는 함수 (매 STEP마다 한 칸)
static void
move_robot_to_one_step(struct robot *robot, int target_row, int target_col)
{
        static const int dr[4] = {-1, 1, 0, 0};
        static const int dc[4] = {0, 0, -1, 1};
        int parent[MAP_HEIGHT][MAP_WIDTH];
        int queue_r[MAP_HEIGHT * MAP_WIDTH];
        int queue_c[MAP_HEIGHT * MAP_WIDTH];
        int path_r[MAP_HEIGHT * MAP_WIDTH];
        int path_c[MAP_HEIGHT * MAP_WIDTH];
        int head = 0;
        int tail = 0;

        if (robot->row == target_row && robot->col == target_col)
                return;

        memset(parent, -1, sizeof parent);
        parent[robot->row][robot->col] = robot->row * MAP_WIDTH + robot->col;
        queue_r[tail] = robot->row;
        queue_c[tail++] = robot->col;

        while (head < tail && parent[target_row][target_col] == -1) {
                int row = queue_r[head];
                int col = queue_c[head++];

                for (int i = 0; i < 4; i++) {
                        int next_row = row + dr[i];
                        int next_col = col + dc[i];

                        if (is_blocked_cell(next_row, next_col) ||
                            parent[next_row][next_col] != -1)
                                continue;

                        parent[next_row][next_col] = row * MAP_WIDTH + col;
                        queue_r[tail] = next_row;
                        queue_c[tail++] = next_col;
                }
        }

        if (parent[target_row][target_col] == -1)
                return;

        // 경로 계산
        int path_len = 0;
        int row = target_row;
        int col = target_col;
        int start = robot->row * MAP_WIDTH + robot->col;

        while (row * MAP_WIDTH + col != start) {
                int prev = parent[row][col];
                path_r[path_len] = row;
                path_c[path_len++] = col;
                row = prev / MAP_WIDTH;
                col = prev % MAP_WIDTH;
        }

        // 경로의 첫 번째 칸으로만 이동 (한 칸만!)
        if (path_len > 0) {
                robot->row = path_r[path_len - 1];
                robot->col = path_c[path_len - 1];
        }
}
// 로봇 스레드 함수
void robot_thread(void* aux) {
        int idx = *((int *)aux);
        struct robot *robot = &robots[idx];
        int item_row;
        int item_col;
        int dest_row;
        int dest_col;
        
        get_item_position(robot->required_item + '0', &item_row, &item_col);
        get_destination_position(robot->destination, &dest_row, &dest_col);

        printf("[%s] 작업 시작: 물건(%d)을 (%d,%d)에서 집어서 (%d,%d)로 이동\n",
               robot->name, robot->required_item, item_row, item_col, dest_row, dest_col);

        // ===== 메시지 시스템 통합 =====
        int task_completed = 0;  // 작업 완료 여부
        
        while(1) {
            // 작업이 아직 안 끝났으면 다음 목표로 한 칸 이동
            if (!task_completed) {
                // 먼저 물건을 집어야 함
                if (robot->current_payload == 0 && 
                    (robot->row != item_row || robot->col != item_col)) {
                    // 물건 위치로 한 칸 이동
                    move_robot_to_one_step(robot, item_row, item_col);
                    printf("[%s] 물건 위치로 이동 중: (%d,%d)\n", 
                           robot->name, robot->row, robot->col);
                } else if (robot->current_payload == 0 && 
                           robot->row == item_row && robot->col == item_col) {
                    // 물건 도착!
                    robot->current_payload = robot->required_payload;
                    printf("[%s] 물건 집음! payload=%d\n", 
                           robot->name, robot->current_payload);
                } else if (robot->current_payload > 0 && 
                           (robot->row != dest_row || robot->col != dest_col)) {
                    // 목적지로 한 칸 이동
                    move_robot_to_one_step(robot, dest_row, dest_col);
                    printf("[%s] 목적지로 이동 중: (%d,%d)\n", 
                           robot->name, robot->row, robot->col);
                } else if (robot->current_payload > 0 && 
                           robot->row == dest_row && robot->col == dest_col) {
                    // 목적지 도착!
                    robot->current_payload = 0;
                    task_completed = 1;
                    printf("[%s] 물건 하역 완료! 작업 끝!\n", robot->name);
                }
            }
            
            // 1️⃣ 자신의 상태를 중앙에 보고
            struct message msg;
            msg.row = robot->row;
            msg.col = robot->col;
            msg.current_payload = robot->current_payload;
            msg.required_payload = robot->required_payload;
            
            printf("[%s] 상태 보고: row=%d, col=%d, payload=%d, completed=%d\n",
                   robot->name, msg.row, msg.col, msg.current_payload, task_completed);
            
            send_message_to_central(idx, &msg);
            
            // 2️⃣ 중앙이 모든 로봇의 상태를 받을 때까지 대기 (BLOCK)
            printf("[%s] 중앙으로부터 명령을 기다리며 BLOCK됩니다.\n", robot->name);
            block_thread();
            printf("[%s] UNBLOCK되었습니다! 명령을 처리합니다.\n", robot->name);
            
            // 3️⃣ 중앙으로부터 받은 명령 실행
            int cmd = receive_command_from_central(idx);
            printf("[%s] 명령 %d을(를) 받았습니다.\n", robot->name, cmd);
            
            // 명령 처리 (현재는 WAIT만 구현)
            if (cmd == 0) {  // WAIT
                printf("[%s] WAIT 명령 실행\n", robot->name);
            }
            
            // 메시지 박스 초기화
            clear_message_box(idx, 0);  // 로봇->중앙 메시지 클리어
        }
}

// entry point of simulator
void run_automated_warehouse(char **argv)
{
        init_automated_warehouse(argv); // do not remove this

        number_of_robots = atoi(argv[1]);
        robots = malloc(sizeof(struct robot) * number_of_robots);

        parse_robot_tasks(argv[2], robots, number_of_robots);

        // 각 로봇에 이름 붙이기
        for (int i = 0; i < number_of_robots; i++) {
            char *name = malloc(10);
            snprintf(name, 10, "R%d", i + 1);
            robots[i].name = name;
        }

        // ===== 디버깅: 파싱 결과 확인 =====
        printf("[DEBUG] Parsing complete!\n");
        printf("[DEBUG] Number of robots: %d\n", number_of_robots);
        for (int i = 0; i < number_of_robots; i++) {
            printf("[DEBUG] Robot %s: Item %d -> Dest %c at (%d,%d)\n",
                   robots[i].name,
                   robots[i].required_item,
                   robots[i].destination,
                   robots[i].row,
                   robots[i].col);
        }
        printf("===================================\n\n");
        
        // ===== 메시지 박스 초기화 =====
        printf("[INFO] 메시지 박스 초기화 중...\n");
        init_message_boxes(number_of_robots);
        printf("[INFO] 메시지 박스 초기화 완료!\n\n");
        
// 중앙 관제 스레드 생성   
        tid_t cnt_thread = thread_create("CNT", 0, central_control_thread, NULL);
        
// 로봇 스레드 생성
        tid_t* robot_threads = malloc(sizeof(tid_t) * number_of_robots);
        int* robot_indices = malloc(sizeof(int) * number_of_robots);

        for (int i = 0; i < number_of_robots; i++) {
            robot_indices[i] = i;  
            robot_threads[i] = thread_create(
                robots[i].name,      
                0,                   
                robot_thread,        
                &robot_indices[i]    
            );
        }

        // must be called after parsing
    handle_parse_completion(cnt_thread, robot_threads, number_of_robots);
        
}
