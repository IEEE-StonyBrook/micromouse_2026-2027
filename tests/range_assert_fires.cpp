// Must NOT compile: a top speed far past the history range trips the
// static_assert in control/encoder_history.h.
#include "config/geometry.h"
#include "config/motion.h"
#undef ROBOT_MAX_WHEEL_SPEED_MMPS
#define ROBOT_MAX_WHEEL_SPEED_MMPS 1.0e7f
#include "control/encoder_history.h"
