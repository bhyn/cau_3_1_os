#include <stdio.h>
#include <string.h>

#include "threads/init.h"
#include "threads/malloc.h"
#include "threads/synch.h"
#include "threads/thread.h"

#include "devices/timer.h"

#include "projects/automated_warehouse/aw_manager.h"

struct robot* robots;

// test code for central control node thread
void test_cnt(){
        while(1){
                print_map(robots, 3);
                thread_sleep(1000);
                block_thread();
        }
}

// test code for robot thread
void test_thread(void* aux){
        int idx = *((int *)aux);
        int test = 0;
        while(1){
                printf("thread %d : %d\n", idx, test++);
                thread_sleep(idx * 1000);
        }
}

// entry point of simulator
void run_automated_warehouse(char **argv)
{
        init_automated_warehouse(argv); // do not remove this

        printf("implement automated warehouse!\n");

        // test case robots
        robots = malloc(sizeof(struct robot) * 3);
        setRobot(&robots[0], "R1", 5, 5, 0, 0);
        setRobot(&robots[1], "R2", 0, 2, 0, 0);
        setRobot(&robots[2], "R3", 1, 1, 1, 1);

        // example of create thread
        tid_t cnt_thread = 0;
        cnt_thread = thread_create("CNT", 0, &test_cnt, NULL);

        tid_t* threads = malloc(sizeof(tid_t) * 3);
        int _idxs[3] = {1, 2, 3};
        threads[0] = thread_create("R1", 0, &test_thread, &_idxs[0]);
        threads[1] = thread_create("R2", 0, &test_thread, &_idxs[1]);
        threads[2] = thread_create("R3", 0, &test_thread, &_idxs[2]);

        // must be called after parsing
        handle_parse_completion(cnt_thread, threads, 3);
        
}