#include <types.h>
#include <lib.h>
#include <synchprobs.h>
#include <synch.h>
#include <opt-A1.h>
/* 
 * This simple default synchronization mechanism allows only vehicle at a time
 * into the intersection.   The intersectionSem is used as a a lock.
 * We use a semaphore rather than a lock so that this code will work even
 * before locks are implemented.
 */

/* 
 * Replace this default synchronization mechanism with your own (better) mechanism
 * needed for your solution.   Your mechanism may use any of the available synchronzation
 * primitives, e.g., semaphores, locks, condition variables.   You are also free to 
 * declare other global variables if your solution requires them.
 */

/*
 * replace this with declarations of any synchronization and other variables you need here
 */
static volatile int n_count, w_count, s_count, e_count = 0;
static struct lock *intersectionLock;
static struct cv *n_cv, *w_cv, *s_cv, *e_cv;
static int cur_dir = 0;
static volatile int intersectionCount = 0;
static volatile bool is_coming = false;
/* 
 * The simulation driver will call this function once before starting
 * the simulation
 *
 * You can use it to initialize synchronization and other variables.
 * 
 */
void
intersection_sync_init(void)
{
  intersectionLock = lock_create("intersectionLock");
  if (intersectionLock == NULL) {
    panic("could not create intersectionLock");
  }

  n_cv = cv_create("n_cv");
  s_cv = cv_create("s_cv");
  w_cv = cv_create("w_cv");
  e_cv = cv_create("e_cv");

  if (n_cv == NULL || s_cv == NULL || w_cv == NULL || e_cv == NULL) {
    panic("could not create condition variables");
  }
  return;
}

/* 
 * The simulation driver will call this function once after
 * the simulation has finished
 *
 * You can use it to clean up any synchronization and other variables.
 *
 */
void
intersection_sync_cleanup(void)
{
  KASSERT(intersectionLock != NULL);
  lock_destroy(intersectionLock);

  KASSERT(n_cv != NULL);
  KASSERT(s_cv != NULL);
  KASSERT(w_cv != NULL);
  KASSERT(e_cv != NULL);
  cv_destroy(n_cv);
  cv_destroy(s_cv);
  cv_destroy(w_cv);
  cv_destroy(e_cv);
}


/*
 * The simulation driver will call this function each time a vehicle
 * tries to enter the intersection, before it enters.
 * This function should cause the calling simulation thread 
 * to block until it is OK for the vehicle to enter the intersection.
*
 * parameters:
 *    * origin: the Direction from which the vehicle is arriving
 *    * destination: the Direction in which the vehicle is trying to go
 *
 * return value: none
 */

void
intersection_before_entry(Direction origin, Direction destination) 
{
    (void)destination; /* avoid compiler complaint about unused parameter */
    // Let the first car enter the intersection
    lock_acquire(intersectionLock);
    KASSERT(intersectionCount >= 0);
    if ((!is_coming) && intersectionCount == 0) {
        ++intersectionCount;
        lock_release(intersectionLock);
        return;
    } else {
        // Block all other cars until the car exists
        // Unblock the direction with the most number of cars
        if (origin == north) {
            ++n_count;
            cv_wait(n_cv, intersectionLock);
            --n_count;
            ++intersectionCount;
        } else if (origin == south) {
            ++s_count;
            cv_wait(s_cv, intersectionLock);
            --s_count;
            ++intersectionCount;
        } else if (origin == west) {
            ++w_count;
            cv_wait(w_cv, intersectionLock);
            --w_count;
            ++intersectionCount;
        } else {
            ++e_count;
            cv_wait(e_cv, intersectionLock);
            --e_count;
            ++intersectionCount;
        }
        lock_release(intersectionLock);
    }
    return;
}


/*
 * The simulation driver will call this function each time a vehicle
 * leaves the intersection.
 *
 * parameters:
 *    * origin: the Direction from which the vehicle arrived
 *    * destination: the Direction in which the vehicle is going
 *
 * return value: none
 */

void
intersection_after_exit(Direction origin, Direction destination) 
{
    (void)destination; /* avoid compiler complaint about unused parameter */
    (void)origin;
    lock_acquire(intersectionLock);
    --intersectionCount;
    if (intersectionCount == 0) {
        // Check all four directions
        int i = 0;
        while (i < 4) {
            cur_dir = (cur_dir + 1) % 4;
            if (cur_dir == 0) {
                // If cur_dir does not have any cars, check the next direction
                if (n_count == 0) {
                    ++i;
                    is_coming = false;
                    continue;
                }
                cv_broadcast(n_cv, intersectionLock);
                is_coming = true;
                break;
            } else if (cur_dir == 1) {
                if (s_count == 0) {
                    ++i;
                    is_coming = false;
                    continue;
                }
                cv_broadcast(s_cv, intersectionLock);
                is_coming = true;
                break;
            } else if (cur_dir == 2) {
                if (w_count == 0) {
                    ++i;
                    is_coming = false;
                    continue;
                }
                cv_broadcast(w_cv, intersectionLock);
                is_coming = true;
                break;
            } else {
                if (e_count == 0) {
                    ++i;
                    is_coming = false;
                    continue;
                }
                cv_broadcast(e_cv, intersectionLock);
                is_coming = true;
                break;
            }
        }
    }
    lock_release(intersectionLock);
}
