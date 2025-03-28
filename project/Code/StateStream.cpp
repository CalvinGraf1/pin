#include "StateStream.h"

Angle degToRad(Angle deg) {
    return deg * M_PI / 180;
}

Angle radToDeg(Angle rad) {
    return rad / M_PI * 180;
}

State calculateNextState(State const &state, Time time) {
    State nextState = state;

    nextState.time = time;

    Time timeRel = time - state.time;

    for (RobotInfo &robot: nextState.robots) {
        if (robot.leftSpeed == robot.rightSpeed) {
            Distance distance = robot.leftSpeed * timeRel;
            robot.position.x += distance * cos(degToRad(robot.angle));
            robot.position.y += distance * sin(degToRad(robot.angle));
        } else {
            Angle alpha = degToRad(robot.angle);
            Distance R = (robot.leftSpeed + robot.rightSpeed) / (robot.leftSpeed - robot.rightSpeed) * robot.radius;
            Angle omega = (robot.leftSpeed - robot.rightSpeed) / (2 * robot.radius);
            std::vector<Distance> vt = {(R * cos(alpha) * sin(omega * timeRel)),
                                        (R * sin(alpha) * sin(omega * timeRel))};
            std::vector<Distance> vr = {(R * sin(alpha) * -1) * (1 - cos(omega * timeRel)),
                                        (R * cos(alpha)) * (1 - cos(omega * timeRel))};

            std::vector<Distance> d = {vt[0] + vr[0], vt[1] + vr[1]};

            robot.position.x += d[0];
            robot.position.y += d[1];
            robot.angle += radToDeg(omega * timeRel);
        }
    }

    return nextState;
}
