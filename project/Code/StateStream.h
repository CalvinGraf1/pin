#ifndef STATESTREAM_H
#define STATESTREAM_H

#include "State.h"
#include "Types.h"
#include <vector>
#include <cmath>
#include <iostream>

Angle degToRad(Angle deg);

Angle radToDeg(Angle rad);

State calculateNextState(State const &state, Time time);

#endif // STATESTREAM_H
