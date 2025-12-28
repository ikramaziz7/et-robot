#include <sys/time.h>
#include <stdint.h>
#include <stdlib.h>

uint32_t millis(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (tv.tv_sec * 1000U) + (tv.tv_usec / 1000U);
}