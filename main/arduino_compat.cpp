#include "esp_timer.h"

unsigned long millis() {
    return static_cast<unsigned long>(esp_timer_get_time() / 1000ULL);
}

unsigned long micros() {
    return static_cast<unsigned long>(esp_timer_get_time());
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}