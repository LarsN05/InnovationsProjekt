#pragma once

#include "line-follower.h"
#include "src/motorshield/motorshield.h"

void initWebInterface(PIDController &pid, Motorshield *shield);

// Returns how many milliseconds the caller should sleep before calling again.
uint32_t updateWebInterface();
