#include "cpusample.h"

volatile std::uint16_t cpu_user::cpu_samples[CPU_TASK_MAX+1] = {0};
std::atomic<sbx_cpu_context_id> cpu_user::m_curr_context;
