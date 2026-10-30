#include "userprog/syscall.h"
#include <stdio.h>
#include <string.h>
#include <syscall-nr.h>
#include "threads/interrupt.h"
#include "threads/thread.h"
#include "threads/loader.h"
#include "threads/init.h"
#include "threads/mmu.h"
#include "threads/palloc.h"
#include "threads/vaddr.h"
#include "userprog/gdt.h"
#include "userprog/process.h"
#include "threads/flags.h"
#include "filesys/file.h"
#include "filesys/filesys.h"
#include "devices/input.h"
#include "intrinsic.h"

void syscall_entry (void);
void syscall_handler (struct intr_frame *);

struct lock filesys_lock;

/* System call.
 *
 * Previously system call services was handled by the interrupt handler
 * (e.g. int 0x80 in linux). However, in x86-64, the manufacturer supplies
 * efficient path for requesting the system call, the `syscall` instruction.
 *
 * The syscall instruction works by reading the values from the the Model
 * Specific Register (MSR). For the details, see the manual. */

#define MSR_STAR 0xc0000081         /* Segment selector msr */
#define MSR_LSTAR 0xc0000082        /* Long mode SYSCALL target */
#define MSR_SYSCALL_MASK 0xc0000084 /* Mask for the eflags */

void
syscall_init (void) {
	write_msr(MSR_STAR, ((uint64_t)SEL_UCSEG - 0x10) << 48  |
			((uint64_t)SEL_KCSEG) << 32);
	write_msr(MSR_LSTAR, (uint64_t) syscall_entry);

	/* The interrupt service rountine should not serve any interrupts
	 * until the syscall_entry swaps the userland stack to the kernel
	 * mode stack. Therefore, we masked the FLAG_FL. */
	write_msr(MSR_SYSCALL_MASK,
			FLAG_IF | FLAG_TF | FLAG_DF | FLAG_IOPL | FLAG_AC | FLAG_NT);

	lock_init (&filesys_lock);
}

/* User memory access.
 *
 * Every pointer a user program hands us is checked before the kernel
 * touches it: it must lie below KERN_BASE and be mapped in the current
 * process's page table.  A bad pointer terminates the process with
 * exit status -1.  The checks run before filesys_lock is acquired, so
 * a process never dies while holding it. */

/* Terminates the process unless the page holding UADDR is mapped
 * (and writable, if WRITABLE) in the current process. */
static void
check_page (const void *uaddr, bool writable) {
	struct thread *curr = thread_current ();
	uint64_t *pte;

	if (uaddr == NULL || !is_user_vaddr (uaddr))
		sys_exit (-1);
	pte = pml4e_walk (curr->pml4, (uint64_t) uaddr, 0);
	if (pte == NULL || (*pte & PTE_P) == 0)
		sys_exit (-1);
	if (writable && (*pte & PTE_W) == 0)
		sys_exit (-1);
}

/* Checks the user buffer [BUFFER, BUFFER + SIZE). */
static void
check_buffer (const void *buffer, size_t size, bool writable) {
	const uint8_t *start = buffer;
	const uint8_t *end = start + size;
	const uint8_t *page;

	if (size == 0)
		return;
	if (end < start)
		sys_exit (-1);
	for (page = pg_round_down (start); page < end; page += PGSIZE)
		check_page (page < start ? start : page, writable);
}

/* Checks the null-terminated user string STR. */
static void
check_string (const char *str) {
	check_page (str, false);
	for (; *str != '\0'; str++)
		if (pg_ofs (str + 1) == 0)
			check_page (str + 1, false);
}

/* Returns the open file of FD in the current process, or NULL. */
static struct file *
fd_to_file (int fd) {
	struct thread *curr = thread_current ();

	if (fd < 2 || fd >= FDT_SIZE)
		return NULL;
	return curr->fdt[fd];
}

/* System call implementations. */

static void
sys_halt (void) {
	power_off ();
}

void
sys_exit (int status) {
	thread_current ()->exit_status = status;
	thread_exit ();
}

static tid_t
sys_fork (const char *thread_name, struct intr_frame *f) {
	check_string (thread_name);
	return process_fork (thread_name, f);
}

static int
sys_exec (const char *cmd_line) {
	char *cmd_copy;

	/* process_exec() destroys the current address space, so the command
	 * line must be copied into the kernel first. */
	check_string (cmd_line);
	cmd_copy = palloc_get_page (0);
	if (cmd_copy == NULL)
		sys_exit (-1);
	strlcpy (cmd_copy, cmd_line, PGSIZE);

	if (process_exec (cmd_copy) < 0)
		sys_exit (-1);
	NOT_REACHED ();
}

static int
sys_wait (tid_t pid) {
	return process_wait (pid);
}

static bool
sys_create (const char *file, unsigned initial_size) {
	bool success;

	check_string (file);
	lock_acquire (&filesys_lock);
	success = filesys_create (file, initial_size);
	lock_release (&filesys_lock);
	return success;
}

static bool
sys_remove (const char *file) {
	bool success;

	check_string (file);
	lock_acquire (&filesys_lock);
	success = filesys_remove (file);
	lock_release (&filesys_lock);
	return success;
}

static int
sys_open (const char *file) {
	struct thread *curr = thread_current ();
	struct file *opened;
	int fd;

	check_string (file);
	for (fd = 2; fd < FDT_SIZE; fd++)
		if (curr->fdt[fd] == NULL)
			break;
	if (fd == FDT_SIZE)
		return -1;

	lock_acquire (&filesys_lock);
	opened = filesys_open (file);
	lock_release (&filesys_lock);
	if (opened == NULL)
		return -1;

	curr->fdt[fd] = opened;
	return fd;
}

static int
sys_filesize (int fd) {
	struct file *file = fd_to_file (fd);
	int size;

	if (file == NULL)
		return -1;
	lock_acquire (&filesys_lock);
	size = file_length (file);
	lock_release (&filesys_lock);
	return size;
}

static int
sys_read (int fd, void *buffer, unsigned size) {
	struct file *file;
	int bytes_read;

	check_buffer (buffer, size, true);
	if (fd == STDIN_FILENO) {
		uint8_t *buf = buffer;
		for (unsigned i = 0; i < size; i++)
			buf[i] = input_getc ();
		return size;
	}

	file = fd_to_file (fd);
	if (file == NULL)
		return -1;
	lock_acquire (&filesys_lock);
	bytes_read = file_read (file, buffer, size);
	lock_release (&filesys_lock);
	return bytes_read;
}

static int
sys_write (int fd, const void *buffer, unsigned size) {
	struct file *file;
	int bytes_written;

	check_buffer (buffer, size, false);
	if (fd == STDOUT_FILENO) {
		putbuf (buffer, size);
		return size;
	}

	file = fd_to_file (fd);
	if (file == NULL)
		return -1;
	lock_acquire (&filesys_lock);
	bytes_written = file_write (file, buffer, size);
	lock_release (&filesys_lock);
	return bytes_written;
}

static void
sys_seek (int fd, unsigned position) {
	struct file *file = fd_to_file (fd);

	if (file == NULL)
		return;
	lock_acquire (&filesys_lock);
	file_seek (file, position);
	lock_release (&filesys_lock);
}

static unsigned
sys_tell (int fd) {
	struct file *file = fd_to_file (fd);
	unsigned position;

	if (file == NULL)
		return -1;
	lock_acquire (&filesys_lock);
	position = file_tell (file);
	lock_release (&filesys_lock);
	return position;
}

static void
sys_close (int fd) {
	struct file *file = fd_to_file (fd);

	if (file == NULL)
		return;
	thread_current ()->fdt[fd] = NULL;
	lock_acquire (&filesys_lock);
	file_close (file);
	lock_release (&filesys_lock);
}

/* The main system call interface.
 * The system call number is in RAX and the arguments are in RDI, RSI,
 * RDX, R10, R8 and R9, in that order.  The return value goes in RAX. */
void
syscall_handler (struct intr_frame *f) {
	uint64_t arg1 = f->R.rdi;
	uint64_t arg2 = f->R.rsi;
	uint64_t arg3 = f->R.rdx;

	switch (f->R.rax) {
		case SYS_HALT:
			sys_halt ();
			break;
		case SYS_EXIT:
			sys_exit ((int) arg1);
			break;
		case SYS_FORK:
			f->R.rax = sys_fork ((const char *) arg1, f);
			break;
		case SYS_EXEC:
			f->R.rax = sys_exec ((const char *) arg1);
			break;
		case SYS_WAIT:
			f->R.rax = sys_wait ((tid_t) arg1);
			break;
		case SYS_CREATE:
			f->R.rax = sys_create ((const char *) arg1, (unsigned) arg2);
			break;
		case SYS_REMOVE:
			f->R.rax = sys_remove ((const char *) arg1);
			break;
		case SYS_OPEN:
			f->R.rax = sys_open ((const char *) arg1);
			break;
		case SYS_FILESIZE:
			f->R.rax = sys_filesize ((int) arg1);
			break;
		case SYS_READ:
			f->R.rax = sys_read ((int) arg1, (void *) arg2, (unsigned) arg3);
			break;
		case SYS_WRITE:
			f->R.rax = sys_write ((int) arg1, (const void *) arg2,
					(unsigned) arg3);
			break;
		case SYS_SEEK:
			sys_seek ((int) arg1, (unsigned) arg2);
			break;
		case SYS_TELL:
			f->R.rax = sys_tell ((int) arg1);
			break;
		case SYS_CLOSE:
			sys_close ((int) arg1);
			break;
		default:
			/* Not implemented yet (e.g. mmap in project 3). */
			sys_exit (-1);
	}
}
