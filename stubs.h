#ifndef SBX_STUBS_H
#define SBX_STUBS_H

#include <sys/types.h>
#include <signal.h>

#ifdef __cplusplus
extern "C" {
#endif

extern void *__dso_handle;

void *_sbrk(int increment);
void _exit(int rc);
void _kill(pid_t pid, int sig);
void __run_init_array(void);

#ifdef __cplusplus
}
#endif

#endif
