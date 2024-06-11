// unblocked로 구현
#include <stdio.h>
#include "threads/thread.h"
#include "threads/synch.h"
#include "projects/crossroads/vehicle.h"
#include "projects/crossroads/map.h"
#include "projects/crossroads/ats.h"

static struct semaphore *crossroad_entrance;
static struct semaphore *cnt;   // 동작하는 스레드 전체 개수


int thread_count;
int unblocked;

/* path. A:0 B:1 C:2 D:3 */
const struct position vehicle_path[4][4][12] = {
    /* from A */ {
        /* to A */
        {{4,0},{4,1},{4,2},{4,3},{4,4},{3,4},{2,4},{2,3},{2,2},{2,1},{2,0},{-1,-1},},
        /* to B */
        {{4,0},{4,1},{4,2},{5,2},{6,2},{-1,-1},},
        /* to C */
        {{4,0},{4,1},{4,2},{4,3},{4,4},{4,5},{4,6},{-1,-1},},
        /* to D */
        {{4,0},{4,1},{4,2},{4,3},{4,4},{3,4},{2,4},{1,4},{0,4},{-1,-1},}
    },
    /* from B */ {
        /* to A */
        {{6,4},{5,4},{4,4},{3,4},{2,4},{2,3},{2,2},{2,1},{2,0},{-1,-1},},
        /* to B */
        {{6,4},{5,4},{4,4},{3,4},{2,4},{2,3},{2,2},{3,2},{4,2},{5,2},{6,2},{-1,-1},},
        /* to C */
        {{6,4},{5,4},{4,4},{4,5},{4,6},{-1,-1},},
        /* to D */
        {{6,4},{5,4},{4,4},{3,4},{2,4},{1,4},{0,4},{-1,-1},}
    },
    /* from C */ {
        /* to A */
        {{2,6},{2,5},{2,4},{2,3},{2,2},{2,1},{2,0},{-1,-1},},
        /* to B */
        {{2,6},{2,5},{2,4},{2,3},{2,2},{3,2},{4,2},{5,2},{6,2},{-1,-1},},
        /* to C */
        {{2,6},{2,5},{2,4},{2,3},{2,2},{3,2},{4,2},{4,3},{4,4},{4,5},{4,6},{-1,-1},},
        /* to D */
        {{2,6},{2,5},{2,4},{1,4},{0,4},{-1,-1},}
    },
    /* from D */ {
        /* to A */
        {{0,2},{1,2},{2,2},{2,1},{2,0},{-1,-1},},
        /* to B */
        {{0,2},{1,2},{2,2},{3,2},{4,2},{5,2},{6,2},{-1,-1},},
        /* to C */
        {{0,2},{1,2},{2,2},{3,2},{4,2},{4,3},{4,4},{4,5},{4,6},{-1,-1},},
        /* to D */
        {{0,2},{1,2},{2,2},{3,2},{4,2},{4,3},{4,4},{3,4},{2,4},{1,4},{0,4},{-1,-1},}
    }
};

static int is_position_outside(struct position pos)
{
    return (pos.row == -1 || pos.col == -1);
}

static int is_position_inside_crossroad(struct position pos)
{
    return (pos.row > 1 && pos.row < 5 && pos.col > 1 && pos.col < 5);
}

/* return 0:termination, 1:success, -1:fail */
static int try_move(int start, int dest, int step, struct vehicle_info *vi)
{
    struct position pos_cur, pos_next;

    pos_next = vehicle_path[start][dest][step];
    pos_cur = vi->position;

    if (vi->state == VEHICLE_STATUS_RUNNING) {
        /* check termination */
        if (is_position_outside(pos_next)) {
            /* actual move */
            vi->position.row = vi->position.col = -1;
            /* release previous */

            if (lock_held_by_current_thread(&vi->map_locks[pos_cur.row][pos_cur.col])) {
                lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
            }
            thread_count--;
            return 0;
        }
    }

    if (!is_position_inside_crossroad(pos_cur) && is_position_inside_crossroad(pos_next)) {
        sema_down(crossroad_entrance);
    }

    if (is_position_inside_crossroad(pos_cur) && !is_position_inside_crossroad(pos_next)) {
        sema_up(crossroad_entrance);
    }

    /* lock next position */

    lock_acquire(&vi->map_locks[pos_next.row][pos_next.col]);

    if (vi->state == VEHICLE_STATUS_READY) {
        /* start this vehicle */
        vi->state = VEHICLE_STATUS_RUNNING;
    } else {
        /* release current position */
        if (lock_held_by_current_thread(&vi->map_locks[pos_cur.row][pos_cur.col])) {
            lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
        }
    }
    /* update position */
    vi->position = pos_next;

    return 1;
}

void init_on_mainthread(int thread_cnt)
{
    /* Called once before spawning threads */
    thread_count = thread_cnt;
    unblocked  = 0;
    crossroad_entrance = (struct semaphore *)malloc(sizeof(struct semaphore));
    cnt = (struct semaphore *)malloc(sizeof(struct semaphore));
    //blocked = (struct semaphore *)malloc(sizeof(struct semaphore));
    sema_init(crossroad_entrance, 7);
    sema_init(cnt, thread_count);
    //sema_init(blocked, 0);
}

void vehicle_loop(void *_vi)
{
    int res;
    int start, dest, step;

    struct vehicle_info *vi = _vi;

    start = vi->start - 'A';
    dest = vi->dest - 'A';

    vi->position.row = vi->position.col = -1;
    vi->state = VEHICLE_STATUS_READY;
    step = 0;
    while (1) {
        if(sema_try_down(cnt)){
            res = try_move(start, dest, step, vi);
            // cnt->value는 블록된 스레드의 개수
            if (res == 1) {
                step++;
            }
            /* termination condition. */ 
            if (res == 0) {
                break;
            }
            unblocked++;
        }
        else{
            crossroads_step++;
            sema_init(cnt, unblocked);
            unblocked = 0;
        }
        /* unitstep change! */
        unitstep_changed();
	}    

    /* status transition must happen before sema_up */
    vi->state = VEHICLE_STATUS_FINISHED;
}

/* 현재 블록되지 않은 스레드의 수를 반환하는 함수 */
int get_unblocked_thread_count(void)
{
    return thread_count - cnt->value;
}
