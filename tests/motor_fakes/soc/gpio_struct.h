#pragma once
#include <cstdint>
struct FakeGpio { uint32_t in=0; struct { uint32_t val=0; } in1; };
extern FakeGpio GPIO;
