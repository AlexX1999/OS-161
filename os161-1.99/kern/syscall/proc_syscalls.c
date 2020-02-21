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
#include "opt-A2.h"

  /* this implementation of sys__exit does not do anything with the exit code */
  /* this needs to be fixed to get exit() and waitpid() working properly */

void sys__exit(int exitcode) {

  struct addrspace *as;
  struct proc *p = curproc;
  /* for now, just include this to keep the compiler from complaining about
     an unused variable */
  (void)exitcode;

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
  proc_destroy(p);
  
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
  *retval = 1;
#if OPT_A2
  *retval = curproc->pid;
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
  /* for now, just pretend the exitstatus is 0 */
  exitstatus = 0;
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
