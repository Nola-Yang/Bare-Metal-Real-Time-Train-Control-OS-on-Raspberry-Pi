#include "kassert.h"

__attribute__((noreturn)) void kassert_fail_kernel(const char *file, int line, const char *cond) {
    panic("KASSERT %s:%d: %s\r\n", file, line, cond);
}
