#ifndef PIN_2023_PARTICLE_H
#define PIN_2023_PARTICLE_H

#include <vector>
#include <cmath>

#include "Position.h"
#include "Types.h"
#include "json_util.h"

struct ParticleInfo {
   UniqueId id; // Unique identifier
   Position position; // (x,y) coordinates of the particle in the world (in pixels)
   Distance radius;  // radius of the circular particle. (pixels)
   std::vector<Times> explosionTimes;
   // times in seconds at which the particle or its children explode into 4
   // smaller particles. the 4 particles centers are at 45, 135, -45 and -135 angles from the original particle
   // center, and their radii are such that their are exactly inscribed in the original particle circle and do
   // not overlap, i.e are equal to 1/(1+sqrt(2))*R where R is the original radius.
   // Note : the score for capturing 4 children is therefore equal to 68.6% of the score for capaturing the
   // particle before it explodes.
   // Note (2) : the last explosion makes the particle disappear entirely. When all particles have disappeared
   // through either explosion or clearing by robots, the game ends.

   NLOHMANN_DEFINE_TYPE_INTRUSIVE(ParticleInfo, id, position, radius, explosionTimes)
   JSON_DEFINE_FLUX_OPERATORS(ParticleInfo)
};

using Particles = std::vector<ParticleInfo>;

#endif //PIN_2023_PARTICLE_H
