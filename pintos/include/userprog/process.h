#ifndef USERPROG_PROCESS_H
#define USERPROG_PROCESS_H

#include "threads/thread.h"
#include "threads/synch.h"

/* Number of entries in a process's file descriptor table (one page).
 * fd 0 and 1 are reserved for the console. */
#define FDT_SIZE 512

/* Exit information of a child process, shared by the parent and the
 * child.  It outlives whichever of the two exits first, so the parent
 * can still wait() for a child that already died, and a child can
 * still exit after its parent is gone. */
struct child_status {
	tid_t tid;                    /* Child's tid. */
	int exit_status;              /* Status the child exited with. */
	int ref_cnt;                  /* Freed when it drops to 0. */
	struct semaphore exit_sema;   /* Upped once the child exits. */
	struct list_elem elem;        /* Element in parent's children list. */
};

tid_t process_create_initd (const char *file_name);
tid_t process_fork (const char *name, struct intr_frame *if_);
int process_exec (void *f_name);
int process_wait (tid_t);
void process_exit (void);
void process_activate (struct thread *next);

#endif /* userprog/process.h */
