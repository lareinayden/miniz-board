#pragma once
#include <cstdint>

struct range_finder_readings_t {
  uint16_t front;
  uint16_t left;
  uint16_t right;
  uint16_t back;
};

void setup_range_finders();
range_finder_readings_t get_latest_range_finder_sample();
void run_range_finder_timer();