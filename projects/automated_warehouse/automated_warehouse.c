#include <stdio.h>
#include <string.h>

#include "threads/init.h"
#include "threads/malloc.h"
#include "threads/synch.h"
#include "threads/thread.h"

#include "devices/timer.h"

#include "projects/automated_warehouse/aw_manager.h"
#include "projects/automated_warehouse/robot.h"

struct robot* robots;
int number_of_robots;


// 중앙 관제 스레드 함수
void central_control_thread(void* aux) {
    while(1){
        print_map(robots, number_of_robots);
        thread_sleep(1000);
        block_thread(); // ?
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

        for (int i = path_len - 1; i >= 0; i--) {
                robot->row = path_r[i];
                robot->col = path_c[i];
                thread_sleep(100);
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

        // 물건 위치로 이동
        move_robot_to(robot, item_row, item_col);
        robot->current_payload = robot->required_payload;
        // 목적지로 이동
        move_robot_to(robot, dest_row, dest_col);
        robot->current_payload = 0;

        block_thread(); // 작업이 끝난 후 스레드를 블록하여 중앙 관제 스레드가 다음 명령을 내릴 때까지 대기
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
