#ifndef TIMELINE_H
#define TIMELINE_H

#include "json_util.h"
#include "State.h"
#include <vector>

struct TimeLine {

 std::vector<State> states;


    NLOHMANN_DEFINE_TYPE_INTRUSIVE(TimeLine,
                                   states)
    JSON_DEFINE_FLUX_OPERATORS(TimeLine)
};

using States = std::vector<State>;
#endif // TIMELINE_H
