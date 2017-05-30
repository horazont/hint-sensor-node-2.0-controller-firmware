#include "stubs.h"

#include <stm32f10x.h>

void *_sbrk(int increment)
{
    (void)increment;
    return (void*)-1;
}

void _exit(int rc)
{
    (void)rc;
    __ASM volatile("bkpt #01");
    while (1);
}

void _kill(pid_t pid, int sig)
{
    (void)pid;
    (void)sig;
    __ASM volatile("bkpt #02");
    while (1);
}

pid_t _getpid()
{
    // WE ARE PID EINS!
    return 1;
}

void *__dso_handle = 0;

// These magic symbols are provided by the linker.
extern void
(*__preinit_array_start[])(void) __attribute__((weak));
extern void
(*__preinit_array_end[])(void) __attribute__((weak));
extern void
(*__init_array_start[])(void) __attribute__((weak));
extern void
(*__init_array_end[])(void) __attribute__((weak));
extern void
(*__fini_array_start[])(void) __attribute__((weak));
extern void
(*__fini_array_end[])(void) __attribute__((weak));

// Iterate over all the preinit/init routines (mainly static constructors).
void __run_init_array(void)
{
  int count;
  int i;

  count = __preinit_array_end - __preinit_array_start;
  for (i = 0; i < count; i++)
    __preinit_array_start[i]();

  // If you need to run the code in the .init section, please use
  // the startup files, since this requires the code in crti.o and crtn.o
  // to add the function prologue/epilogue.
  //_init(); // DO NOT ENABE THIS!

  count = __init_array_end - __init_array_start;
  for (i = 0; i < count; i++)
    __init_array_start[i]();
}
