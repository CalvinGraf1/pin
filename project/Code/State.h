#ifndef PIN_2023_STATE_H
#define PIN_2023_STATE_H

#include "json_util.h"
#include <vector>
#include "Robot.h"
#include "Particle.h"

struct State {
   Time time;
   Position worldOrigin;
   Position worldEnd;
   Robots robots;
   Particles particles;

   NLOHMANN_DEFINE_TYPE_INTRUSIVE(State,
                                  time,
                                  worldOrigin,
                                  worldEnd,
                                  robots,
                                  particles)
   JSON_DEFINE_FLUX_OPERATORS(State)


};

using States = std::vector<State>;


#endif //PIN_2023_STATE_H
