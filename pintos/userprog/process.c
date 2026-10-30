#include "userprog/process.h"
#include <debug.h>
#include <inttypes.h>
#include <round.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "userprog/gdt.h"
#include "userprog/syscall.h"
#include "userprog/tss.h"
#include "filesys/directory.h"
#include "filesys/file.h"
#include "filesys/filesys.h"
#include "threads/flags.h"
#include "threads/init.h"
#include "threads/interrupt.h"
#include "threads/malloc.h"
#include "threads/palloc.h"
#include "threads/thread.h"
#include "threads/mmu.h"
#include "threads/vaddr.h"
#include "intrinsic.h"
#ifdef VM
#include "vm/vm.h"
#endif

/* Maximum number of words on a command line. */
#define ARGV_MAX 64

static void process_cleanup (void);
static bool load (const char *file_name, struct intr_frame *if_);
static bool push_arguments (struct intr_frame *if_, int argc, char **argv);
static void initd (void *aux);
static void __do_fork (void *);

/* Argument of initd(). */
struct initd_aux {
	char *file_name;              /* Command line, in its own page. */
	struct child_status *cs;      /* Shared with the creator. */
};

/* Argument of __do_fork().  Lives on the parent's stack, which is safe
 * because the parent sleeps on DONE until the child is done with it. */
struct fork_aux {
	struct thread *parent;
	struct intr_frame *parent_if; /* Parent's user context at fork(). */
	struct child_status *cs;      /* Shared with the parent. */
	struct semaphore done;        /* Upped once duplication finishes. */
	bool success;                 /* Whether duplication succeeded. */
};

/* Allocates a child_status referenced by both parent and child. */
static struct child_status *
child_status_create (void) {
	struct child_status *cs = malloc (sizeof *cs);
	if (cs == NULL)
		return NULL;
	cs->tid = TID_ERROR;
	cs->exit_status = -1;
	cs->ref_cnt = 2;
	sema_init (&cs->exit_sema, 0);
	return cs;
}

/* Drops one reference to CS, freeing it when nobody refers to it.
 * The parent and the child may call this at the same time. */
static void
child_status_release (struct child_status *cs) {
	enum intr_level old_level = intr_disable ();
	int ref_cnt = --cs->ref_cnt;
	intr_set_level (old_level);

	if (ref_cnt == 0)
		free (cs);
}

/* General process initializer for initd and other process.
 * Allocates the file descriptor table.  Returns false on failure. */
static bool
process_init (void) {
	struct thread *current = thread_current ();

	current->fdt = palloc_get_page (PAL_ZERO);
	return current->fdt != NULL;
}

/* Starts the first userland program, called "initd", loaded from FILE_NAME.
 * The new thread may be scheduled (and may even exit)
 * before process_create_initd() returns. Returns the initd's
 * thread id, or TID_ERROR if the thread cannot be created.
 * Notice that THIS SHOULD BE CALLED ONCE. */
tid_t
process_create_initd (const char *file_name) {
	char *fn_copy;
	char prog_name[16];
	struct initd_aux *aux;
	tid_t tid;

	/* Make a copy of FILE_NAME.
	 * Otherwise there's a race between the caller and load(). */
	fn_copy = palloc_get_page (0);
	if (fn_copy == NULL)
		return TID_ERROR;
	strlcpy (fn_copy, file_name, PGSIZE);

	aux = malloc (sizeof *aux);
	if (aux == NULL) {
		palloc_free_page (fn_copy);
		return TID_ERROR;
	}
	aux->file_name = fn_copy;
	aux->cs = child_status_create ();
	if (aux->cs == NULL) {
		free (aux);
		palloc_free_page (fn_copy);
		return TID_ERROR;
	}
	struct child_status *cs = aux->cs;

	/* The thread is named after the program, not the whole command line. */
	strlcpy (prog_name, file_name, sizeof prog_name);
	prog_name[strcspn (prog_name, " ")] = '\0';

	/* Create a new thread to execute FILE_NAME. */
	tid = thread_create (prog_name, PRI_DEFAULT, initd, aux);
	if (tid == TID_ERROR) {
		free (cs);
		free (aux);
		palloc_free_page (fn_copy);
		return TID_ERROR;
	}
	cs->tid = tid;
	list_push_back (&thread_current ()->children, &cs->elem);
	return tid;
}

/* A thread function that launches first user process. */
static void
initd (void *aux_) {
	struct initd_aux *aux = aux_;
	char *f_name = aux->file_name;

	thread_current ()->my_status = aux->cs;
	free (aux);

#ifdef VM
	supplemental_page_table_init (&thread_current ()->spt);
#endif

	if (!process_init ())
		PANIC("Fail to launch initd\n");

	if (process_exec (f_name) < 0)
		PANIC("Fail to launch initd\n");
	NOT_REACHED ();
}

/* Clones the current process as `name`. Returns the new process's thread id, or
 * TID_ERROR if the thread cannot be created. */
tid_t
process_fork (const char *name, struct intr_frame *if_) {
	struct fork_aux aux;
	tid_t tid;

	aux.parent = thread_current ();
	aux.parent_if = if_;
	aux.success = false;
	sema_init (&aux.done, 0);
	aux.cs = child_status_create ();
	if (aux.cs == NULL)
		return TID_ERROR;

	/* Clone current thread to new thread.*/
	tid = thread_create (name, PRI_DEFAULT, __do_fork, &aux);
	if (tid == TID_ERROR) {
		free (aux.cs);
		return TID_ERROR;
	}
	aux.cs->tid = tid;
	list_push_back (&aux.parent->children, &aux.cs->elem);

	/* Do not return until the child has duplicated our resources. */
	sema_down (&aux.done);
	if (!aux.success) {
		list_remove (&aux.cs->elem);
		child_status_release (aux.cs);
		return TID_ERROR;
	}
	return tid;
}

#ifndef VM
/* Duplicate the parent's address space by passing this function to the
 * pml4_for_each. This is only for the project 2. */
static bool
duplicate_pte (uint64_t *pte, void *va, void *aux) {
	struct thread *current = thread_current ();
	struct thread *parent = (struct thread *) aux;
	void *parent_page;
	void *newpage;
	bool writable;

	/* 1. If the parent_page is kernel page, then return immediately. */
	if (is_kernel_vaddr (va))
		return true;

	/* 2. Resolve VA from the parent's page map level 4. */
	parent_page = pml4_get_page (parent->pml4, va);
	if (parent_page == NULL)
		return true;

	/* 3. Allocate new PAL_USER page for the child and set result to
	 *    NEWPAGE. */
	newpage = palloc_get_page (PAL_USER);
	if (newpage == NULL)
		return false;

	/* 4. Duplicate parent's page to the new page and
	 *    check whether parent's page is writable or not (set WRITABLE
	 *    according to the result). */
	memcpy (newpage, parent_page, PGSIZE);
	writable = (*pte & PTE_W) != 0;

	/* 5. Add new page to child's page table at address VA with WRITABLE
	 *    permission. */
	if (!pml4_set_page (current->pml4, va, newpage, writable)) {
		/* 6. if fail to insert page, do error handling. */
		palloc_free_page (newpage);
		return false;
	}
	return true;
}
#endif

/* A thread function that copies parent's execution context.
 * Hint) parent->tf does not hold the userland context of the process.
 *       That is, you are required to pass second argument of process_fork to
 *       this function. */
static void
__do_fork (void *aux_) {
	struct fork_aux *aux = aux_;
	struct intr_frame if_;
	struct thread *parent = aux->parent;
	struct thread *current = thread_current ();
	struct intr_frame *parent_if = aux->parent_if;
	bool succ = true;

	current->my_status = aux->cs;

	/* 1. Read the cpu context to local stack. */
	memcpy (&if_, parent_if, sizeof (struct intr_frame));
	if_.R.rax = 0;              /* fork() returns 0 in the child. */

	/* 2. Duplicate PT */
	current->pml4 = pml4_create();
	if (current->pml4 == NULL)
		goto error;

	process_activate (current);
#ifdef VM
	supplemental_page_table_init (&current->spt);
	if (!supplemental_page_table_copy (&current->spt, &parent->spt))
		goto error;
#else
	if (!pml4_for_each (parent->pml4, duplicate_pte, parent))
		goto error;
#endif

	if (!process_init ())
		goto error;

	/* 3. Duplicate the open files, including the executable.  The parent
	 *    is blocked in process_fork(), so its table does not change. */
	lock_acquire (&filesys_lock);
	for (int fd = 2; fd < FDT_SIZE && succ; fd++) {
		if (parent->fdt[fd] == NULL)
			continue;
		current->fdt[fd] = file_duplicate (parent->fdt[fd]);
		if (current->fdt[fd] == NULL)
			succ = false;
	}
	if (succ && parent->running_file != NULL) {
		current->running_file = file_duplicate (parent->running_file);
		if (current->running_file == NULL)
			succ = false;
	}
	lock_release (&filesys_lock);

	/* Finally, switch to the newly created process. */
	if (succ) {
		/* AUX is gone once the parent wakes up, so this is its last use. */
		aux->success = true;
		sema_up (&aux->done);
		do_iret (&if_);
	}
error:
	sema_up (&aux->done);
	thread_exit ();
}

/* Switch the current execution context to the f_name.
 * Returns -1 on fail. */
int
process_exec (void *f_name) {
	struct thread *curr = thread_current ();
	char *file_name = f_name;
	char *argv[ARGV_MAX];
	char *token, *save_ptr;
	int argc = 0;
	bool success;

	/* Split the command line into words.  Multiple spaces count as one.
	 * The words stay in FILE_NAME's page, which ARGV points into. */
	for (token = strtok_r (file_name, " ", &save_ptr); token != NULL;
			token = strtok_r (NULL, " ", &save_ptr)) {
		if (argc == ARGV_MAX) {
			palloc_free_page (file_name);
			return -1;
		}
		argv[argc++] = token;
	}
	if (argc == 0) {
		palloc_free_page (file_name);
		return -1;
	}

	/* We cannot use the intr_frame in the thread structure.
	 * This is because when current thread rescheduled,
	 * it stores the execution information to the member. */
	struct intr_frame _if;
	_if.ds = _if.es = _if.ss = SEL_UDSEG;
	_if.cs = SEL_UCSEG;
	_if.eflags = FLAG_IF | FLAG_MBS;

	/* We first kill the current context */
	process_cleanup ();

	/* The old executable is replaced, so it may be written again. */
	if (curr->running_file != NULL) {
		lock_acquire (&filesys_lock);
		file_close (curr->running_file);
		lock_release (&filesys_lock);
		curr->running_file = NULL;
	}

	/* And then load the binary */
	success = load (argv[0], &_if) && push_arguments (&_if, argc, argv);

	/* If load failed, quit. */
	palloc_free_page (file_name);
	if (!success)
		return -1;

	/* Start switched process. */
	do_iret (&_if);
	NOT_REACHED ();
}

/* Pushes the ARGC words of ARGV onto the user stack that load() set up,
 * and passes them to the program's entry function as in the x86-64
 * calling convention: RDI = argc, RSI = argv.  Each ARGV[i] is
 * overwritten with the user address of its copy.  Returns false if
 * the arguments do not fit in the single stack page. */
static bool
push_arguments (struct intr_frame *if_, int argc, char **argv) {
	uintptr_t rsp = if_->rsp;
	uintptr_t stack_bottom = USER_STACK - PGSIZE;
	int i;

	/* The words themselves. */
	for (i = argc - 1; i >= 0; i--) {
		size_t len = strlen (argv[i]) + 1;
		if (rsp - stack_bottom < len)
			return false;
		rsp -= len;
		memcpy ((void *) rsp, argv[i], len);
		argv[i] = (char *) rsp;
	}

	/* Word-align, then argv[argc] = NULL, argv[argc - 1] ... argv[0],
	 * and a fake return address.  The page is zeroed, so the padding
	 * is already 0. */
	rsp = ROUND_DOWN (rsp, sizeof (char *));
	if (rsp - stack_bottom < (argc + 2) * sizeof (char *))
		return false;

	rsp -= sizeof (char *);
	*(char **) rsp = NULL;
	for (i = argc - 1; i >= 0; i--) {
		rsp -= sizeof (char *);
		*(char **) rsp = argv[i];
	}
	if_->R.rdi = argc;
	if_->R.rsi = rsp;

	rsp -= sizeof (void *);
	*(void **) rsp = NULL;
	if_->rsp = rsp;
	return true;
}


/* Waits for thread TID to die and returns its exit status.  If
 * it was terminated by the kernel (i.e. killed due to an
 * exception), returns -1.  If TID is invalid or if it was not a
 * child of the calling process, or if process_wait() has already
 * been successfully called for the given TID, returns -1
 * immediately, without waiting.
 *
 * Only direct children are found in the children list, and a child
 * is removed from it once waited for. */
int
process_wait (tid_t child_tid) {
	struct thread *curr = thread_current ();
	struct list_elem *e;

	for (e = list_begin (&curr->children); e != list_end (&curr->children);
			e = list_next (e)) {
		struct child_status *cs = list_entry (e, struct child_status, elem);
		if (cs->tid == child_tid) {
			int status;

			sema_down (&cs->exit_sema);
			status = cs->exit_status;
			list_remove (&cs->elem);
			child_status_release (cs);
			return status;
		}
	}
	return -1;
}

/* Exit the process. This function is called by thread_exit (). */
void
process_exit (void) {
	struct thread *curr = thread_current ();

	/* Only user processes have a page table; kernel threads stay quiet. */
	if (curr->pml4 != NULL)
		printf ("%s: exit(%d)\n", curr->name, curr->exit_status);

	/* Close every open file, including the executable. */
	if (curr->fdt != NULL || curr->running_file != NULL) {
		lock_acquire (&filesys_lock);
		if (curr->fdt != NULL)
			for (int fd = 2; fd < FDT_SIZE; fd++)
				file_close (curr->fdt[fd]);
		file_close (curr->running_file);
		lock_release (&filesys_lock);

		if (curr->fdt != NULL)
			palloc_free_page (curr->fdt);
		curr->fdt = NULL;
		curr->running_file = NULL;
	}

	/* Children are not inherited, so just forget them. */
	while (!list_empty (&curr->children)) {
		struct child_status *cs = list_entry (list_pop_front (&curr->children),
				struct child_status, elem);
		child_status_release (cs);
	}

	/* Hand the exit status to the parent, if it still cares. */
	if (curr->my_status != NULL) {
		curr->my_status->exit_status = curr->exit_status;
		sema_up (&curr->my_status->exit_sema);
		child_status_release (curr->my_status);
		curr->my_status = NULL;
	}

	process_cleanup ();
}

/* Free the current process's resources. */
static void
process_cleanup (void) {
	struct thread *curr = thread_current ();

#ifdef VM
	supplemental_page_table_kill (&curr->spt);
#endif

	uint64_t *pml4;
	/* Destroy the current process's page directory and switch back
	 * to the kernel-only page directory. */
	pml4 = curr->pml4;
	if (pml4 != NULL) {
		/* Correct ordering here is crucial.  We must set
		 * cur->pagedir to NULL before switching page directories,
		 * so that a timer interrupt can't switch back to the
		 * process page directory.  We must activate the base page
		 * directory before destroying the process's page
		 * directory, or our active page directory will be one
		 * that's been freed (and cleared). */
		curr->pml4 = NULL;
		pml4_activate (NULL);
		pml4_destroy (pml4);
	}
}

/* Sets up the CPU for running user code in the nest thread.
 * This function is called on every context switch. */
void
process_activate (struct thread *next) {
	/* Activate thread's page tables. */
	pml4_activate (next->pml4);

	/* Set thread's kernel stack for use in processing interrupts. */
	tss_update (next);
}

/* We load ELF binaries.  The following definitions are taken
 * from the ELF specification, [ELF1], more-or-less verbatim.  */

/* ELF types.  See [ELF1] 1-2. */
#define EI_NIDENT 16

#define PT_NULL    0            /* Ignore. */
#define PT_LOAD    1            /* Loadable segment. */
#define PT_DYNAMIC 2            /* Dynamic linking info. */
#define PT_INTERP  3            /* Name of dynamic loader. */
#define PT_NOTE    4            /* Auxiliary info. */
#define PT_SHLIB   5            /* Reserved. */
#define PT_PHDR    6            /* Program header table. */
#define PT_STACK   0x6474e551   /* Stack segment. */

#define PF_X 1          /* Executable. */
#define PF_W 2          /* Writable. */
#define PF_R 4          /* Readable. */

/* Executable header.  See [ELF1] 1-4 to 1-8.
 * This appears at the very beginning of an ELF binary. */
struct ELF64_hdr {
	unsigned char e_ident[EI_NIDENT];
	uint16_t e_type;
	uint16_t e_machine;
	uint32_t e_version;
	uint64_t e_entry;
	uint64_t e_phoff;
	uint64_t e_shoff;
	uint32_t e_flags;
	uint16_t e_ehsize;
	uint16_t e_phentsize;
	uint16_t e_phnum;
	uint16_t e_shentsize;
	uint16_t e_shnum;
	uint16_t e_shstrndx;
};

struct ELF64_PHDR {
	uint32_t p_type;
	uint32_t p_flags;
	uint64_t p_offset;
	uint64_t p_vaddr;
	uint64_t p_paddr;
	uint64_t p_filesz;
	uint64_t p_memsz;
	uint64_t p_align;
};

/* Abbreviations */
#define ELF ELF64_hdr
#define Phdr ELF64_PHDR

static bool setup_stack (struct intr_frame *if_);
static bool validate_segment (const struct Phdr *, struct file *);
static bool load_segment (struct file *file, off_t ofs, uint8_t *upage,
		uint32_t read_bytes, uint32_t zero_bytes,
		bool writable);

/* Loads an ELF executable from FILE_NAME into the current thread.
 * Stores the executable's entry point into *RIP
 * and its initial stack pointer into *RSP.
 * Returns true if successful, false otherwise. */
static bool
load (const char *file_name, struct intr_frame *if_) {
	struct thread *t = thread_current ();
	struct ELF ehdr;
	struct file *file = NULL;
	off_t file_ofs;
	bool success = false;
	int i;

	lock_acquire (&filesys_lock);

	/* Allocate and activate page directory. */
	t->pml4 = pml4_create ();
	if (t->pml4 == NULL)
		goto done;
	process_activate (thread_current ());

	/* Open executable file. */
	file = filesys_open (file_name);
	if (file == NULL) {
		printf ("load: %s: open failed\n", file_name);
		goto done;
	}

	/* Read and verify executable header. */
	if (file_read (file, &ehdr, sizeof ehdr) != sizeof ehdr
			|| memcmp (ehdr.e_ident, "\177ELF\2\1\1", 7)
			|| ehdr.e_type != 2
			|| ehdr.e_machine != 0x3E // amd64
			|| ehdr.e_version != 1
			|| ehdr.e_phentsize != sizeof (struct Phdr)
			|| ehdr.e_phnum > 1024) {
		printf ("load: %s: error loading executable\n", file_name);
		goto done;
	}

	/* Read program headers. */
	file_ofs = ehdr.e_phoff;
	for (i = 0; i < ehdr.e_phnum; i++) {
		struct Phdr phdr;

		if (file_ofs < 0 || file_ofs > file_length (file))
			goto done;
		file_seek (file, file_ofs);

		if (file_read (file, &phdr, sizeof phdr) != sizeof phdr)
			goto done;
		file_ofs += sizeof phdr;
		switch (phdr.p_type) {
			case PT_NULL:
			case PT_NOTE:
			case PT_PHDR:
			case PT_STACK:
			default:
				/* Ignore this segment. */
				break;
			case PT_DYNAMIC:
			case PT_INTERP:
			case PT_SHLIB:
				goto done;
			case PT_LOAD:
				if (validate_segment (&phdr, file)) {
					bool writable = (phdr.p_flags & PF_W) != 0;
					uint64_t file_page = phdr.p_offset & ~PGMASK;
					uint64_t mem_page = phdr.p_vaddr & ~PGMASK;
					uint64_t page_offset = phdr.p_vaddr & PGMASK;
					uint32_t read_bytes, zero_bytes;
					if (phdr.p_filesz > 0) {
						/* Normal segment.
						 * Read initial part from disk and zero the rest. */
						read_bytes = page_offset + phdr.p_filesz;
						zero_bytes = (ROUND_UP (page_offset + phdr.p_memsz, PGSIZE)
								- read_bytes);
					} else {
						/* Entirely zero.
						 * Don't read anything from disk. */
						read_bytes = 0;
						zero_bytes = ROUND_UP (page_offset + phdr.p_memsz, PGSIZE);
					}
					if (!load_segment (file, file_page, (void *) mem_page,
								read_bytes, zero_bytes, writable))
						goto done;
				}
				else
					goto done;
				break;
		}
	}

	/* Set up stack. */
	if (!setup_stack (if_))
		goto done;

	/* Start address. */
	if_->rip = ehdr.e_entry;

	/* Arguments are pushed by process_exec(), see push_arguments(). */

	success = true;

done:
	/* We arrive here whether the load is successful or not.
	 * On success, keep the executable open and deny writes to it
	 * while the process runs; process_exit() closes it. */
	if (success) {
		file_deny_write (file);
		t->running_file = file;
	} else
		file_close (file);
	lock_release (&filesys_lock);
	return success;
}


/* Checks whether PHDR describes a valid, loadable segment in
 * FILE and returns true if so, false otherwise. */
static bool
validate_segment (const struct Phdr *phdr, struct file *file) {
	/* p_offset and p_vaddr must have the same page offset. */
	if ((phdr->p_offset & PGMASK) != (phdr->p_vaddr & PGMASK))
		return false;

	/* p_offset must point within FILE. */
	if (phdr->p_offset > (uint64_t) file_length (file))
		return false;

	/* p_memsz must be at least as big as p_filesz. */
	if (phdr->p_memsz < phdr->p_filesz)
		return false;

	/* The segment must not be empty. */
	if (phdr->p_memsz == 0)
		return false;

	/* The virtual memory region must both start and end within the
	   user address space range. */
	if (!is_user_vaddr ((void *) phdr->p_vaddr))
		return false;
	if (!is_user_vaddr ((void *) (phdr->p_vaddr + phdr->p_memsz)))
		return false;

	/* The region cannot "wrap around" across the kernel virtual
	   address space. */
	if (phdr->p_vaddr + phdr->p_memsz < phdr->p_vaddr)
		return false;

	/* Disallow mapping page 0.
	   Not only is it a bad idea to map page 0, but if we allowed
	   it then user code that passed a null pointer to system calls
	   could quite likely panic the kernel by way of null pointer
	   assertions in memcpy(), etc. */
	if (phdr->p_vaddr < PGSIZE)
		return false;

	/* It's okay. */
	return true;
}

#ifndef VM
/* Codes of this block will be ONLY USED DURING project 2.
 * If you want to implement the function for whole project 2, implement it
 * outside of #ifndef macro. */

/* load() helpers. */
static bool install_page (void *upage, void *kpage, bool writable);

/* Loads a segment starting at offset OFS in FILE at address
 * UPAGE.  In total, READ_BYTES + ZERO_BYTES bytes of virtual
 * memory are initialized, as follows:
 *
 * - READ_BYTES bytes at UPAGE must be read from FILE
 * starting at offset OFS.
 *
 * - ZERO_BYTES bytes at UPAGE + READ_BYTES must be zeroed.
 *
 * The pages initialized by this function must be writable by the
 * user process if WRITABLE is true, read-only otherwise.
 *
 * Return true if successful, false if a memory allocation error
 * or disk read error occurs. */
static bool
load_segment (struct file *file, off_t ofs, uint8_t *upage,
		uint32_t read_bytes, uint32_t zero_bytes, bool writable) {
	ASSERT ((read_bytes + zero_bytes) % PGSIZE == 0);
	ASSERT (pg_ofs (upage) == 0);
	ASSERT (ofs % PGSIZE == 0);

	file_seek (file, ofs);
	while (read_bytes > 0 || zero_bytes > 0) {
		/* Do calculate how to fill this page.
		 * We will read PAGE_READ_BYTES bytes from FILE
		 * and zero the final PAGE_ZERO_BYTES bytes. */
		size_t page_read_bytes = read_bytes < PGSIZE ? read_bytes : PGSIZE;
		size_t page_zero_bytes = PGSIZE - page_read_bytes;

		/* Get a page of memory. */
		uint8_t *kpage = palloc_get_page (PAL_USER);
		if (kpage == NULL)
			return false;

		/* Load this page. */
		if (file_read (file, kpage, page_read_bytes) != (int) page_read_bytes) {
			palloc_free_page (kpage);
			return false;
		}
		memset (kpage + page_read_bytes, 0, page_zero_bytes);

		/* Add the page to the process's address space. */
		if (!install_page (upage, kpage, writable)) {
			printf("fail\n");
			palloc_free_page (kpage);
			return false;
		}

		/* Advance. */
		read_bytes -= page_read_bytes;
		zero_bytes -= page_zero_bytes;
		upage += PGSIZE;
	}
	return true;
}

/* Create a minimal stack by mapping a zeroed page at the USER_STACK */
static bool
setup_stack (struct intr_frame *if_) {
	uint8_t *kpage;
	bool success = false;

	kpage = palloc_get_page (PAL_USER | PAL_ZERO);
	if (kpage != NULL) {
		success = install_page (((uint8_t *) USER_STACK) - PGSIZE, kpage, true);
		if (success)
			if_->rsp = USER_STACK;
		else
			palloc_free_page (kpage);
	}
	return success;
}

/* Adds a mapping from user virtual address UPAGE to kernel
 * virtual address KPAGE to the page table.
 * If WRITABLE is true, the user process may modify the page;
 * otherwise, it is read-only.
 * UPAGE must not already be mapped.
 * KPAGE should probably be a page obtained from the user pool
 * with palloc_get_page().
 * Returns true on success, false if UPAGE is already mapped or
 * if memory allocation fails. */
static bool
install_page (void *upage, void *kpage, bool writable) {
	struct thread *t = thread_current ();

	/* Verify that there's not already a page at that virtual
	 * address, then map our page there. */
	return (pml4_get_page (t->pml4, upage) == NULL
			&& pml4_set_page (t->pml4, upage, kpage, writable));
}
#else
/* From here, codes will be used after project 3.
 * If you want to implement the function for only project 2, implement it on the
 * upper block. */

static bool
lazy_load_segment (struct page *page, void *aux) {
	/* TODO: Load the segment from the file */
	/* TODO: This called when the first page fault occurs on address VA. */
	/* TODO: VA is available when calling this function. */
}

/* Loads a segment starting at offset OFS in FILE at address
 * UPAGE.  In total, READ_BYTES + ZERO_BYTES bytes of virtual
 * memory are initialized, as follows:
 *
 * - READ_BYTES bytes at UPAGE must be read from FILE
 * starting at offset OFS.
 *
 * - ZERO_BYTES bytes at UPAGE + READ_BYTES must be zeroed.
 *
 * The pages initialized by this function must be writable by the
 * user process if WRITABLE is true, read-only otherwise.
 *
 * Return true if successful, false if a memory allocation error
 * or disk read error occurs. */
static bool
load_segment (struct file *file, off_t ofs, uint8_t *upage,
		uint32_t read_bytes, uint32_t zero_bytes, bool writable) {
	ASSERT ((read_bytes + zero_bytes) % PGSIZE == 0);
	ASSERT (pg_ofs (upage) == 0);
	ASSERT (ofs % PGSIZE == 0);

	while (read_bytes > 0 || zero_bytes > 0) {
		/* Do calculate how to fill this page.
		 * We will read PAGE_READ_BYTES bytes from FILE
		 * and zero the final PAGE_ZERO_BYTES bytes. */
		size_t page_read_bytes = read_bytes < PGSIZE ? read_bytes : PGSIZE;
		size_t page_zero_bytes = PGSIZE - page_read_bytes;

		/* TODO: Set up aux to pass information to the lazy_load_segment. */
		void *aux = NULL;
		if (!vm_alloc_page_with_initializer (VM_ANON, upage,
					writable, lazy_load_segment, aux))
			return false;

		/* Advance. */
		read_bytes -= page_read_bytes;
		zero_bytes -= page_zero_bytes;
		upage += PGSIZE;
	}
	return true;
}

/* Create a PAGE of stack at the USER_STACK. Return true on success. */
static bool
setup_stack (struct intr_frame *if_) {
	bool success = false;
	void *stack_bottom = (void *) (((uint8_t *) USER_STACK) - PGSIZE);

	/* TODO: Map the stack on stack_bottom and claim the page immediately.
	 * TODO: If success, set the rsp accordingly.
	 * TODO: You should mark the page is stack. */
	/* TODO: Your code goes here */

	return success;
}
#endif /* VM */
