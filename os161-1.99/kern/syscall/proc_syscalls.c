#include <types.h>
#include <kern/errno.h>
#include <kern/unistd.h>
#include <kern/wait.h>
#include <lib.h>
#include <syscall.h>
#include <current.h>
#include <proc.h>
#include <thread.h>
#include <addrspace.h>
#include <copyinout.h>
#include <../arch/mips/include/trapframe.h>
#include <synch.h>
#include "opt-A2.h"

  /* this implementation of sys__exit does not do anything with the exit code */
  /* this needs to be fixed to get exit() and waitpid() working properly */

void sys__exit(int exitcode) {

  struct addrspace *as;
  // struct proc *p = curproc;

#if OPT_A2
  curproc->exitcode = exitcode;
  curproc->is_dead = true;
  // Parent has called wait_pid on curproc
  if (curproc->proc_lk != NULL && curproc->proc_cv != NULL) {
    cv_signal(curproc->proc_cv, curproc->proc_lk);
  }
#else
  /* for now, just include this to keep the compiler from complaining about
     an unused variable */
  (void)exitcode;
#endif

  DEBUG(DB_SYSCALL,"Syscall: _exit(%d)\n",exitcode);

  KASSERT(curproc->p_addrspace != NULL);
  as_deactivate();
  /*
   * clear p_addrspace before calling as_destroy. Otherwise if
   * as_destroy sleeps (which is quite possible) when we
   * come back we'll be calling as_activate on a
   * half-destroyed address space. This tends to be
   * messily fatal.
   */
  as = curproc_setas(NULL);
  as_destroy(as);

  /* detach this thread from its process */
  /* note: curproc cannot be used after this call */
  proc_remthread(curthread);

  /* if this is the last user process in the system, proc_destroy()
     will wake up the kernel menu thread */
  //proc_destroy(p);
  
  thread_exit();
  /* thread_exit() does not return, so we should never get here */
  panic("return from thread_exit in sys_exit\n");
}


/* stub handler for getpid() system call                */
int
sys_getpid(pid_t *retval)
{
  /* for now, this is just a stub that always returns a PID of 1 */
  /* you need to fix this to make it work properly */
#if OPT_A2
  *retval = curproc->pid;
#else
  *retval = 1;
#endif
  return(0);
}

/* stub handler for waitpid() system call                */

int
sys_waitpid(pid_t pid,
	    userptr_t status,
	    int options,
	    pid_t *retval)
{
  int exitstatus;
  int result;

  /* this is just a stub implementation that always reports an
     exit status of 0, regardless of the actual exit status of
     the specified process.   
     In fact, this will return 0 even if the specified process
     is still running, and even if it never existed in the first place.

     Fix this!
  */

  if (options != 0) {
    return(EINVAL);
  }
#if OPT_A2
  // Check if called on child
  struct proc *wait_child = NULL;
  int i = 0;
  int max = array_num(curproc->children);
  while (i < max) {
    struct proc *child_ptr = (struct proc *)array_get(curproc->children, i);
    if (child_ptr->pid == pid) {
      wait_child = child_ptr;
      break;
    }
    ++i;
  }
  if (wait_child == NULL) {
    return EINVAL;
  }
  
  // If the child is dead
  if (wait_child->is_dead) {
    exitstatus = _MKWAIT_EXIT(wait_child->exitcode);
    proc_destroy(wait_child);
  } else {
    // If the child is alive
    KASSERT(wait_child->proc_cv == NULL);
    wait_child->proc_cv = cv_create("parent's bed");

    KASSERT(wait_child->proc_lk == NULL);
    wait_child->proc_lk = lock_create("parent's bed");

    if (wait_child->proc_cv == NULL || wait_child->proc_lk == NULL) {
      if (wait_child->proc_cv != NULL) {
        cv_destroy(wait_child->proc_cv);
      }
      if (wait_child->proc_lk != NULL) {
        lock_destroy(wait_child->proc_lk);
      }
      return ENOMEM;
    }
    
    // Put parent proc to sleep
    lock_acquire(curproc->proc_lk);
    cv_wait(wait_child->proc_cv, wait_child->proc_lk);

    // After signaled by its child
    exitstatus = _MKWAIT_EXIT(wait_child->exitcode);

    cv_destroy(wait_child->proc_cv);
    lock_destroy(wait_child->proc_lk);
    proc_destroy(wait_child);
  }
#else
  /* for now, just pretend the exitstatus is 0 */
  exitstatus = 0;
#endif
  result = copyout((void *)&exitstatus,status,sizeof(int));
  if (result) {
    return(result);
  }
  *retval = pid;
  return(0);
}

#if OPT_A2
int
proc_fork(struct trapframe *tf, pid_t *ret_pid) 
{
    // Create child process
    struct proc *child = NULL;
    child = proc_create_runprogram("DEADBEEF");
    if (child == NULL) {
        return ENOMEM;
    }
    *ret_pid = child->pid;

    // Copy address space
    if (curproc->p_addrspace == NULL) {
        return EFAULT;
    }
    int retval;
    struct addrspace *child_addrspace;
    retval = as_copy(curproc->p_addrspace, &child_addrspace);
    if (retval) {
        return retval;
    }
    spinlock_acquire(&child->p_lock);
    child->p_addrspace = child_addrspace;
    spinlock_release(&child->p_lock);

    // Establish child and parent relationship
    child->parent_proc = curproc;
    if (curproc->children == NULL) {
        curproc->children = array_create();
        if (curproc->children == NULL) {
            return ENOMEM;
        }
    }
    int add_child_result = array_add(curproc->children, child, NULL);
    if (add_child_result) {
        return add_child_result;
    }
    
    struct trapframe *heap_tf = kmalloc(sizeof(*tf));
    *heap_tf = *tf;
    // Create a thread for child proc to run
    thread_fork("Entrypoint", child, &enter_forked_process, heap_tf, 0);
    
    return 0;
}
#endif
