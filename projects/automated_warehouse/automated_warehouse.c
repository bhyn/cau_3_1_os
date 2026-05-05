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


// 중앙 관제 스레드 함수 (나중에 구현)
void central_control_thread(void* aux) {
    // TODO: 중앙 관제 로직 구현
}

// 로봇 스레드 함수 (나중에 구현)
void robot_thread(void* aux) {
    int idx = *((int *)aux);
    // TODO: 로봇 로직 구현
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
            sprintf(name, "R%d", i + 1);
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