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
