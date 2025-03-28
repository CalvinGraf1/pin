#ifndef ROBOTS_POSITION_H
#define ROBOTS_POSITION_H

#include "json_util.h"

struct Position {
   double x;
   double y;

   NLOHMANN_DEFINE_TYPE_INTRUSIVE(Position, x, y)
   JSON_DEFINE_FLUX_OPERATORS(Position)

public:
   friend double distanceBetween(Position const& p, Position const& q) {
      return hypot(p.x - q.x, p.y - q.y);
   }
};

#endif //ROBOTS_POSITION_H
