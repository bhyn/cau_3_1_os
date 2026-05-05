#include "projects/automated_warehouse/aw_thread.h"

#include <stdio.h>

//
// You need to read carefully threads/synch.h and threads/synch.c
// 
// In the code, a fucntion named "sema_down" implements blocking thread and 
// makes list of blocking thread
// 
// And a function named "sema_up" implements unblocing thread using blocking list
//
// You must implement blocking list using "blocking_threads" in this code.
// Then you can also implement unblocking thread.
//


struct list blocked_threads;

/**
 * A function unblocking all blocked threads in "blocked_threads" 
 * It must be called by robot threads
 */
void block_thread(){
    // block 시킨 스레드를 struct list blocked_threads에 넣어야 함

    // 스케줄링을 하는 동안에는 인터럽트가 안되게 해야함 
    enum intr_level old_level; // 현재 인터럽트 상태를 저장할 변수
    struct thread *cur = thread_current();

    printf("[DEBUG] %s 스레드를 blocked_threads에 넣기 전입니다. 현재 대기 스레드 수: %d\n",
           cur->name, list_size(&blocked_threads));

    old_level = intr_disable (); // 인터럽트를 비활성화하여 선점 방지
    list_push_back(&blocked_threads, &cur->elem); // 블록된 스레드를 리스트에 추가

    printf("[DEBUG] %s 스레드를 blocked_threads에 넣었습니다. 현재 대기 스레드 수: %d\n",
           cur->name, list_size(&blocked_threads));
    printf("[DEBUG] %s 스레드를 BLOCKED 상태로 바꿉니다.\n", cur->name);

    thread_block (); // 현재 스레드를 블록 상태로 변경
    intr_set_level (old_level); // 인터럽트 상태를 원래대로 복원

    printf("[DEBUG] %s 스레드가 다시 깨어났습니다.\n", cur->name);
}

/**
 * A function unblocking all blocked threads in "blocked_threads" 
 * It must be called by central control thread
 */
void unblock_threads(){
    // struct list blocked_threads에 있는 모든 스레드를 unblock 시켜야 함
    
    // 스케줄링을 하는 동안에는 인터럽트가 안되게 해야함 
    enum intr_level old_level;

    printf("[DEBUG] unblock_threads가 호출되었습니다. 깨울 스레드 수: %d\n",
           list_size(&blocked_threads));

    old_level = intr_disable(); // 인터럽트를 비활성화하여 선점 방지

    while (!list_empty(&blocked_threads)) {
        struct list_elem *e = list_pop_front(&blocked_threads);
        struct thread *t = list_entry(e, struct thread, elem);

        printf("[DEBUG] %s 스레드를 READY 상태로 깨웁니다. 남은 대기 스레드 수: %d\n",
               t->name, list_size(&blocked_threads));

        thread_unblock(t);
    }

    intr_set_level(old_level); // 인터럽트 상태를 원래대로 복원

    printf("[DEBUG] unblock_threads 처리가 끝났습니다.\n");
}
