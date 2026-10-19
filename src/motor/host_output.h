#pragma once
// Nonblocking on ESP32: the communications loop drains an output queue.
void hostPrintf(const char* format, ...);
