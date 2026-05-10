#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "projects/automated_warehouse/automated_warehouse.h"

#include "threads/init.h"
#include "threads/malloc.h"
#include "threads/synch.h"
#include "threads/thread.h"

#include "devices/shutdown.h"
#include "devices/timer.h"

#include "projects/automated_warehouse/aw_manager.h"
#include "projects/automated_warehouse/robot.h"
#include "projects/automated_warehouse/aw_message.h"
#include "projects/automated_warehouse/aw_thread.h"

struct robot* robots;
int number_of_robots;

#define MAX_ROBOTS 100
#define ROBOT_NAME_SIZE 16
static int current_row[MAX_ROBOTS];
static int current_col[MAX_ROBOTS];
static int target_row[MAX_ROBOTS];
static int target_col[MAX_ROBOTS];
static int desired_cmd[MAX_ROBOTS];
static int next_row[MAX_ROBOTS];
static int next_col[MAX_ROBOTS];
static int accepted[MAX_ROBOTS];
static int considered[MAX_ROBOTS];
static int wait_steps[MAX_ROBOTS];
static int sacrifice_robot_idx = -1;

// Forward declarations
static int is_blocked_cell(struct robot *robot, int row, int col);
static int is_shared_place(int row, int col);
static void get_item_position(char item, int *row, int *col);
static void get_destination_position(char destination, int *row, int *col);
static void get_robot_target(struct robot *robot, int *row, int *col);
static int get_next_direction(struct robot *robot, int target_row, int target_col);
static int get_escape_direction(struct robot *robot, int target_row, int target_col);
static int get_distance(int row, int col, int target_row, int target_col);
static void get_next_position(int row, int col, int direction,
                              int *next_row, int *next_col);
static int choose_deadlock_victim(int n_robots, int *victim_direction);
static int can_accept_batch_move(int robot_idx, int accepted[],
                                 int current_row[], int current_col[],
                                 int next_row[], int next_col[]);

// 중앙 관제 스레드 함수
static void
central_control_thread(void* aux)
{
    int n_robots = number_of_robots;
    (void) aux;

    for (int i = 0; i < n_robots; i++)
        wait_steps[i] = 0;

    while (1) {
        // 1. 모든 로봇으로부터 상태 메시지 수집
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
                timer_sleep(10);
                timeout++;
            }
        }

        for (int i = 0; i < n_robots; i++) {
            struct message msg = receive_message_from_robot(i);
            robots[i].row = msg.row;
            robots[i].col = msg.col;
            robots[i].current_payload = msg.current_payload;
            robots[i].required_payload = msg.required_payload;
            robots[i].task_completed = msg.completed;
        }

        // 2. 맵 출력
        print_map(robots, n_robots);

        // 3. 모든 로봇이 작업을 완료했는지 확인
        int all_completed = 1;
        for (int i = 0; i < n_robots; i++) {
            // 모든 로봇이 작업을 완료했고 물건을 들고 있지 않아야 함
            if (robots[i].task_completed == 0 || robots[i].current_payload != 0) {
                all_completed = 0;
                break;
            }
        }

        if (all_completed) {
            shutdown_power_off();
        }

        // 먼저 모든 로봇에게 기본 WAIT 명령 전송
        for (int i = 0; i < n_robots; i++) {
            send_command_to_robot(i, 0);
        }

        int accepted_count = 0;
        int used_deadlock_victim = 0;

        // ============================================
        // 1단계: 모든 로봇의 목표, 현재위치, 다음위치 계산
        // ============================================
        for (int i = 0; i < n_robots; i++) {
            current_row[i] = robots[i].row;
            current_col[i] = robots[i].col;
            target_row[i] = -1;
            target_col[i] = -1;
            desired_cmd[i] = 0;
            next_row[i] = robots[i].row;
            next_col[i] = robots[i].col;
            accepted[i] = 0;
            considered[i] = 0;

            if (!robots[i].task_completed) {
                // 로봇의 목표 위치 결정 (물건 또는 하역장소)
                get_robot_target(&robots[i], &target_row[i], &target_col[i]);

                // 희생 로봇이면 이동 명령 없음 (데드락 탈출 중)
                if (i == sacrifice_robot_idx) {
                    desired_cmd[i] = 0;
                } else {
                    // BFS로 목표까지의 다음 방향 계산
                    desired_cmd[i] = get_next_direction(&robots[i],
                                                        target_row[i],
                                                        target_col[i]);
                }
                // 명령을 실행했을 때의 다음 위치 미리 계산
                get_next_position(robots[i].row, robots[i].col,
                                  desired_cmd[i], &next_row[i], &next_col[i]);
            }
        }

        // ============================================
        // 2단계: 오래 기다린 로봇부터 이동 가능 여부 판단
        // ============================================
        for (int checked = 0; checked < n_robots; checked++) {
            int robot_idx = -1;
            int best_wait = -1;

            for (int i = 0; i < n_robots; i++) {
                if (considered[i] || robots[i].task_completed)
                    continue;

                if (robot_idx == -1 || wait_steps[i] > best_wait) {
                    robot_idx = i;
                    best_wait = wait_steps[i];
                }
            }

            if (robot_idx == -1)
                break;

            considered[robot_idx] = 1;

            // 이동 가능 조건:
            // 1) 로봇이 이동하려고 함 (desired_cmd != 0)
            // 2) 다른 로봇들과의 충돌 없음 (can_accept_batch_move)
            if (desired_cmd[robot_idx] != 0 &&
                can_accept_batch_move(robot_idx, accepted,
                                      current_row, current_col,
                                      next_row, next_col)) {
                accepted[robot_idx] = 1;
                accepted_count++;
            }
        }

        // ============================================
        // 3단계: 아무도 이동하지 못했으면 데드락
        //        희생 로봇 선택 후 뒤로 빠지게 함
        // ============================================
        if (accepted_count == 0) {
            int victim_idx;
            int victim_direction = 0;

            // 희생 로봇을 선택해서 목표에서 멀어지는 방향으로 이동
            victim_idx = choose_deadlock_victim(n_robots, &victim_direction);

            if (victim_idx != -1 && victim_direction != 0) {
                // 희생 로봇의 명령을 뒤로 가는 방향으로 변경
                desired_cmd[victim_idx] = victim_direction;
                get_next_position(robots[victim_idx].row,
                                  robots[victim_idx].col,
                                  victim_direction,
                                  &next_row[victim_idx],
                                  &next_col[victim_idx]);
                accepted[victim_idx] = 1;
                accepted_count = 1;
                used_deadlock_victim = 1;

            }
        } else {
            // 이동이 있었으면 희생 로봇 상태 초기화
            sacrifice_robot_idx = -1;
        }

        // ============================================
        // 4단계: accepted 로봇들에게만 실제 명령 전송
        // ============================================
        if (accepted_count > 0) {

            for (int i = 0; i < n_robots; i++) {
                if (!accepted[i])
                    continue;


                send_command_to_robot(i, desired_cmd[i]);
            }
        } else {
            // 아무도 이동하지 못했으면 모든 로봇 WAIT
            if (!used_deadlock_victim)
                sacrifice_robot_idx = -1;
        }

        for (int i = 0; i < n_robots; i++) {
            if (robots[i].task_completed || accepted[i])
                wait_steps[i] = 0;
            else
                wait_steps[i]++;
        }

        // 5. 스텝 증가
        increase_step();

        // 6. 모든 로봇 깨우기 (UNBLOCK)
        unblock_threads();

        timer_sleep(10);
    }
}
// 셀에 장애물이나 다른 로봇이 있는지 확인하는 함수
// 반환: 1이면 이동 불가, 0이면 이동 가능
static int
is_blocked_cell(struct robot *robot, int row, int col)
{
        // 경계 확인
        if (row < 0 || row >= MAP_HEIGHT || col < 0 || col >= MAP_WIDTH)
                return 1;

        // 맵의 장애물(X) 확인
        if (map_draw_default[row][col] == 'X')
                return 1;

        // 다른 로봇이 있는지 확인
        // 단, 하역 장소(A, B, C)와 대기 장소(W)에서는 중복 가능
        for (int i = 0; i < number_of_robots; i++) {
                if (robots[i].row == row && robots[i].col == col) {
                        // 로봇이 있는 위치 확인
                        char cell = map_draw_default[row][col];

                        // 하역 장소(A, B, C) 또는 대기 장소(W)라면 중복 가능
                        if (cell == 'A' || cell == 'B' || cell == 'C' || cell == 'W')
                                return 0;

                        // 그 외 장소에서는 로봇이 있으면 불가
                        return 1;
                }
        }

        // 적재하러 가는 자기 물건 칸만 진입 가능
        char cell = map_draw_default[row][col];
        if (cell >= '1' && cell <= '9') {
                if (robot->current_payload == 0 &&
                    cell == robot->required_item + '0')
                        return 0;
                return 1;
        }

        return 0;  // 이동 가능
}

static int
is_shared_place(int row, int col)
{
        char cell = map_draw_default[row][col];

        return cell == 'A' || cell == 'B' || cell == 'C' || cell == 'W';
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

static void
get_robot_target(struct robot *robot, int *row, int *col)
{
        if (robot->current_payload == 0)
                get_item_position(robot->required_item + '0', row, col);
        else
                get_destination_position(robot->destination, row, col);
}

// BFS를 이용하여 다음 이동 방향 계산 (로봇 현재 위치에서 목표 위치로)
// 반환: 0=WAIT, 1=UP, 2=DOWN, 3=LEFT, 4=RIGHT
static int
get_next_direction(struct robot *robot, int target_row, int target_col)
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

        memset(parent, -1, sizeof parent);
        parent[robot->row][robot->col] = robot->row * MAP_WIDTH + robot->col;
        queue_r[tail] = robot->row;
        queue_c[tail++] = robot->col;

        while (head < tail) {
                int row = queue_r[head];
                int col = queue_c[head++];

                for (int i = 0; i < 4; i++) {
                        int next_row = row + dr[i];
                        int next_col = col + dc[i];

                        if (is_blocked_cell(robot, next_row, next_col) ||
                            parent[next_row][next_col] != -1)
                                continue;

                        parent[next_row][next_col] = row * MAP_WIDTH + col;
                        queue_r[tail] = next_row;
                        queue_c[tail++] = next_col;
                }
        }

        if (parent[target_row][target_col] == -1) {
                int best_row = robot->row;
                int best_col = robot->col;
                int best_distance = get_distance(robot->row, robot->col,
                                                 target_row, target_col);

                for (int r = 0; r < MAP_HEIGHT; r++) {
                        for (int c = 0; c < MAP_WIDTH; c++) {
                                int distance;

                                if (parent[r][c] == -1)
                                        continue;

                                distance = get_distance(r, c,
                                                        target_row, target_col);
                                if (distance < best_distance) {
                                        best_distance = distance;
                                        best_row = r;
                                        best_col = c;
                                }
                        }
                }

                if (best_row == robot->row && best_col == robot->col) {
                        return 0;
                }

                target_row = best_row;
                target_col = best_col;
        }

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

        // 경로의 첫 번째 칸으로 가는 방향 반환
        if (path_len > 0) {
                int next_row = path_r[path_len - 1];
                int next_col = path_c[path_len - 1];

                // 다음 위치와 현재 위치를 비교하여 방향 결정
                if (next_row < robot->row) return 1;  // UP
                if (next_row > robot->row) return 2;  // DOWN
                if (next_col < robot->col) return 3;  // LEFT
                if (next_col > robot->col) return 4;  // RIGHT
        }

        return 0;  // WAIT
}

static int
get_escape_direction(struct robot *robot, int target_row, int target_col)
{
        static const int dr[5] = {0, -1, 1, 0, 0};
        static const int dc[5] = {0, 0, 0, -1, 1};
        int best_direction = 0;
        int best_score = -1000000;
        int current_distance = get_distance(robot->row, robot->col,
                                            target_row, target_col);

        for (int direction = 1; direction <= 4; direction++) {
                int next_row = robot->row + dr[direction];
                int next_col = robot->col + dc[direction];
                int next_distance;
                int score;
                char cell;

                if (is_blocked_cell(robot, next_row, next_col))
                        continue;

                next_distance = get_distance(next_row, next_col,
                                             target_row, target_col);
                if (next_distance < current_distance)
                        continue;

                score = (next_distance - current_distance) * 100;
                cell = map_draw_default[next_row][next_col];

                if (is_shared_place(next_row, next_col))
                        score += 100;
                else if (cell == 'S')
                        score += 20;
                else if (cell == ' ')
                        score += 10;

                if (score > best_score) {
                        best_score = score;
                        best_direction = direction;
                }
        }

        return best_direction;
}

static int
get_distance(int row, int col, int target_row, int target_col)
{
        int row_distance = row - target_row;
        int col_distance = col - target_col;

        if (row_distance < 0)
                row_distance = -row_distance;
        if (col_distance < 0)
                col_distance = -col_distance;

        return row_distance + col_distance;
}

static void
get_next_position(int row, int col, int direction,
                  int *next_row, int *next_col)
{
        *next_row = row;
        *next_col = col;

        if (direction == 1)
                (*next_row)--;
        else if (direction == 2)
                (*next_row)++;
        else if (direction == 3)
                (*next_col)--;
        else if (direction == 4)
                (*next_col)++;
}

static int
choose_deadlock_victim(int n_robots, int *victim_direction)
{
        int direction;
        int victim_idx = -1;
        int victim_wait = -1;

        *victim_direction = 0;

        // 이미 정한 희생 로봇이 계속 뒤로 빠질 수 있으면 그대로 유지한다.
        if (sacrifice_robot_idx >= 0 &&
            sacrifice_robot_idx < n_robots &&
            !robots[sacrifice_robot_idx].task_completed) {
                direction = get_escape_direction(&robots[sacrifice_robot_idx],
                                                 target_row[sacrifice_robot_idx],
                                                 target_col[sacrifice_robot_idx]);
                if (direction != 0) {
                        *victim_direction = direction;
                        return sacrifice_robot_idx;
                }
        }

        sacrifice_robot_idx = -1;

        // escape 가능한 로봇 중 가장 오래 기다린 로봇을 희생 로봇으로 고른다.
        for (int robot_idx = 0; robot_idx < n_robots; robot_idx++) {
                if (robots[robot_idx].task_completed)
                        continue;

                direction = get_escape_direction(&robots[robot_idx],
                                                 target_row[robot_idx],
                                                 target_col[robot_idx]);
                if (direction != 0 && wait_steps[robot_idx] > victim_wait) {
                        victim_idx = robot_idx;
                        victim_wait = wait_steps[robot_idx];
                        *victim_direction = direction;
                }
        }

        sacrifice_robot_idx = victim_idx;
        return victim_idx;
}

static int
can_accept_batch_move(int robot_idx, int accepted[],
                      int current_row[], int current_col[],
                      int next_row[], int next_col[])
{
        // 충돌 감지: 현재 검토하는 로봇이 다른 로봇들과 안전하게 이동할 수 있는지 확인
        for (int i = 0; i < number_of_robots; i++) {
                if (i == robot_idx)
                        continue;

                if (accepted[i]) {
                        // 이미 이동 승인된 로봇과의 충돌 확인
                        // 규칙: current/next 좌표가 겹치면 안 됨

                        // 1) 같은 현재 위치 (불가능)
                        if (current_row[robot_idx] == current_row[i] &&
                            current_col[robot_idx] == current_col[i])
                                return 0;

                        // 2) 내 현재 위치 = 상대 다음 위치 (불가능)
                        if (current_row[robot_idx] == next_row[i] &&
                            current_col[robot_idx] == next_col[i])
                                return 0;

                        // 3) 내 다음 위치 = 상대 현재 위치 (불가능)
                        if (next_row[robot_idx] == current_row[i] &&
                            next_col[robot_idx] == current_col[i])
                                return 0;

                        // 4) 내 다음 위치 = 상대 다음 위치 (불가능)
                        if (next_row[robot_idx] == next_row[i] &&
                            next_col[robot_idx] == next_col[i])
                                return 0;
                } else {
                        // 아직 결정되지 않은 로봇 (현재 대기 중)
                        // 규칙: 공유 장소가 아닌 일반 칸에 있는 로봇이 있으면 진입 불가
                        if (!is_shared_place(next_row[robot_idx],
                                             next_col[robot_idx]) &&
                            next_row[robot_idx] == current_row[i] &&
                            next_col[robot_idx] == current_col[i])
                                return 0;
                }
        }

        return 1;  // 충돌 없음 → 이동 가능
}

// 로봇을 명령받은 방향으로 한 칸 이동
// direction: 0=WAIT, 1=UP, 2=DOWN, 3=LEFT, 4=RIGHT
static void
move_robot_one_step_by_direction(struct robot *robot, int direction)
{
        if (direction == 0) return;  // WAIT - 이동하지 않음

        int next_row = robot->row;
        int next_col = robot->col;

        if (direction == 1) next_row--;       // UP
        else if (direction == 2) next_row++;  // DOWN
        else if (direction == 3) next_col--;  // LEFT
        else if (direction == 4) next_col++;  // RIGHT

        if (!is_blocked_cell(robot, next_row, next_col)) {
                robot->row = next_row;
                robot->col = next_col;
        }

}
// 로봇 스레드 함수
static void
robot_thread(void* aux)
{
        int idx = *((int *)aux);
        struct robot *robot = &robots[idx];
        int item_row;
        int item_col;
        int dest_row;
        int dest_col;

        get_item_position(robot->required_item + '0', &item_row, &item_col);
        get_destination_position(robot->destination, &dest_row, &dest_col);

        robot->task_completed = 0;

        while (1) {
            // 작업 진행 상황에 따른 상태 메시지 작성
            struct message msg;
            msg.row = robot->row;
            msg.col = robot->col;
            msg.current_payload = robot->current_payload;
            msg.required_payload = robot->required_payload;
            msg.completed = robot->task_completed;

            send_message_to_central(idx, &msg);

            // 중앙이 모든 로봇의 상태를 받을 때까지 대기 (BLOCK)
            block_thread();

            // 중앙으로부터 받은 명령 실행
            int cmd = receive_command_from_central(idx);
            clear_message_box(idx, 1);

            if (!robot->task_completed) {
                if (cmd != 0)
                    move_robot_one_step_by_direction(robot, cmd);

                if (robot->current_payload == 0 &&
                    robot->row == item_row &&
                    robot->col == item_col) {
                    robot->current_payload = robot->required_payload;
                } else if (robot->current_payload > 0 &&
                           robot->row == dest_row &&
                           robot->col == dest_col) {
                    robot->current_payload = 0;
                    robot->task_completed = 1;
                }
            }


            // 메시지 박스 초기화
            clear_message_box(idx, 0);
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
            char *name = malloc(ROBOT_NAME_SIZE);
            snprintf(name, ROBOT_NAME_SIZE, "R%d", i + 1);
            robots[i].name = name;
        }

        // ===== 메시지 박스 초기화 =====
        init_message_boxes(number_of_robots);

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
