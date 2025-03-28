#include "Simu.h"
#include "Position.h"

#include <climits>
#include <algorithm>

using namespace std;

void Simu::setFirstState(State const &state) {
    timeline.states.clear();
    timeline.states.push_back(state);
}

void Simu::setConstraints(Constraints const &_constraints) {
    constraints = _constraints;
    STEP_VALUE = constraints.commandTimeInterval /2.5;
}

State Simu::getLastState() const {
    return timeline.states.back();
}

TimeLine Simu::getTimeline() const {
    return this->timeline;
}

bool Simu::verifyParticlesExplosion(State &state, Time step, int &lastParticleId) {
    bool newStateDetected = false;
    while (step >= state.particles.back().explosionTimes[0][0]) {
        vector<ParticleInfo> newParticles = explodeParticle(state.particles.back(),
                                                            lastParticleId);

        state.particles.pop_back();

        for (auto &p: newParticles) {
            state.particles.push_back(p);
        }

        sort(state.particles.begin(), state.particles.end(), explosionTimeIsGreater);

        newStateDetected = true;

        if (state.particles.empty()) {
            break;
        }
    }
    return newStateDetected;
}

bool Simu::verifyRobotsCollision(State &state, const State &previousState) {
    bool newStateDetected = false;
    for (size_t i = 0; i < state.robots.size() - 1; ++i) {
        for (size_t j = i + 1; j < state.robots.size(); ++j) {
            RobotInfo r1 = state.robots[i];
            RobotInfo r2 = state.robots[j];
            if (r1.leftSpeed == 0 and r1.rightSpeed == 0 and
                r2.leftSpeed == 0 and r2.rightSpeed == 0) {
                continue;
            }
            if (distanceBetween(r1.position, r2.position) <= r1.radius + r2.radius) {
                state = dichotomicSearch<RobotInfo>(previousState, previousState.time,
                                                    STEP_VALUE,
                                                    i, j);
                newStateDetected = true;
            }
        }
    }
    return newStateDetected;
}

bool Simu::verifyRobotsCollisionSmall(State const &state) {
    bool newStateDetected = false;
    for (size_t i = 0; i < state.robots.size() - 1; ++i) {
        for (size_t j = i + 1; j < state.robots.size(); ++j) {
            RobotInfo r1 = state.robots[i];
            RobotInfo r2 = state.robots[j];
            if (distanceBetween(r1.position, r2.position) <= r1.radius + r2.radius) {
                newStateDetected = true;
            }
        }
    }
    return newStateDetected;
}

bool Simu::verifyRobotsParticlesCollision(State &state, const State &previousState) {
    bool newStateDetected = false;
    for (size_t i = 0; i < state.robots.size(); ++i) {
        RobotInfo r = state.robots[i];
        if (r.leftSpeed == 0 and r.rightSpeed == 0) {
            continue;
        }
        for (size_t j = 0; j < state.particles.size(); ++j) {
            ParticleInfo p = state.particles[j];

            if (distanceBetween(r.position, p.position) <= r.radius + p.radius) {
                state = dichotomicSearch<ParticleInfo>(previousState, previousState.time,
                                                       STEP_VALUE,
                                                       i, j);
                robotEatParticle.emplace_back(r, p);
                newStateDetected = true;
            }
        }
    }
    return newStateDetected;
}

void Simu::calculateTimeline(State state) {
    sort(state.particles.begin(), state.particles.end(), explosionTimeIsGreater);

    Time step = state.time;
    int lastParticleId = state.particles.size();

    bool newStateDetected = false;
    while (not state.particles.empty()) {
        newStateDetected = false;

        State previousState = state;
        step += STEP_VALUE;

        // Find next robots position
        state = calculateNextState(state, step);
        // Particle explosion verification
        if (verifyParticlesExplosion(state, step, lastParticleId))
            newStateDetected = true;


        // Decontamination verification
        if (eatParticle(state))
            newStateDetected = true;

        // Collisions verification
        if (not state.robots.empty()) {
            if (verifyRobotsCollision(state, previousState))
                newStateDetected = true;
            if (verifyRobotsParticlesCollision(state, previousState))
                newStateDetected = true;
        }

        // Calculate the direction of the robot
        if (fmod(step, constraints.commandTimeInterval) <= EPSILON) {
            particleNear(state);

            if (calculateDirectionRobot(state, step))
                newStateDetected = true;
        }

        if (newStateDetected) {
            addState(state);
        }
    }
}

bool Simu::eatParticle(State &state) {
    bool newStateDetected = false;
    vector<pair<RobotInfo, ParticleInfo>> tempRobotEatParticle = robotEatParticle;
    long long counter = 0;
    while (not tempRobotEatParticle.empty()) {
        if (isEatable(tempRobotEatParticle.back().first, tempRobotEatParticle.back().second)) {
            newStateDetected = true;

            int id = tempRobotEatParticle.back().first.id;
            for (auto &robot: state.robots) {
                if (robot.id == id) {
                    robot.idParticle = -1;
                    robot.score += M_PI * pow(robot.radius, 2);
                    break;
                }
            }

            int particleId = tempRobotEatParticle.back().second.id;
            state.particles.erase(
                    remove_if(state.particles.begin(), state.particles.end(),
                              [particleId](const ParticleInfo &p) {
                                  return p.id == particleId;
                              }), state.particles.end());

            robotEatParticle.erase(robotEatParticle.begin() + counter);


            ++counter;
        }
        tempRobotEatParticle.pop_back();
    }
    return newStateDetected;
}

void Simu::addState(const State &state) {
    timeline.states.push_back(state);
}

std::vector<ParticleInfo>
Simu::explodeParticle(const ParticleInfo &particleToExplode, int &lastParticleId) {
    vector<ParticleInfo> newParticles;

    // If the particle is too small, it does not multiply
    if (particleToExplode.explosionTimes.size() > 1) {
        // Create an array of relative positions for the new particles
        int positions[4][2] = {{-1, 1},
                               {-1, -1},
                               {1,  1},
                               {1,  -1}};

        double radius = 1 / (1 + sqrt(2)) * particleToExplode.radius;

        // Creation of the 4 new particles
        for (int i = 0; i < 4; ++i) {
            ParticleInfo newParticle;
            newParticle.id = lastParticleId;
            newParticle.radius = radius;

            // Calculate the new position using values from the positions array
            newParticle.position.x =
                    particleToExplode.position.x + positions[i][0] * radius;
            newParticle.position.y =
                    particleToExplode.position.y + positions[i][1] * radius;

            // Get the explosion times for the new particle and their children
            vector<Times> newExplosionTimes;

            for (size_t j = 1; j < particleToExplode.explosionTimes.size(); ++j) {
                Times explosionTimes;
                size_t childrenSize = pow(4, j - 1);

                for (size_t k = i * childrenSize; k < (i + 1) * childrenSize; ++k) {
                    explosionTimes.push_back(particleToExplode.explosionTimes[j][k]);
                }
                newExplosionTimes.push_back(explosionTimes);
            }
            newParticle.explosionTimes = newExplosionTimes;

            newParticles.push_back(newParticle);
            ++lastParticleId;
        }
    }

    return newParticles;
}

Distance Simu::distanceBetween(const Position &pos1, const Position &pos2) {
    return sqrt(pow(pos1.x - pos2.x, 2) + pow(pos1.y - pos2.y, 2));
//    return distance <= radius1 + radius2; //TODO put variable EPSILON
}

Angle Simu::angleBetween(Position posRobot, Position posParticle) {
    double deltaX = posRobot.x - posParticle.x, deltaY =
            posRobot.y - posParticle.y;

    Angle alpha;
    if (deltaX == 0) {
        if (deltaY > 0) {
            alpha = 270;
        } else {
            alpha = 90;
        }
    } else if (deltaY == 0) {
        if (deltaX > 0) {
            alpha = 180;
        } else {
            alpha = 0;
        }
    } else {
        alpha = abs(radToDeg(::atan(deltaY / deltaX)));


        if (deltaX > 0 and deltaY > 0) {
            alpha += 180;
        } else if (deltaX < 0 and deltaY > 0) {
            alpha = 360 - alpha;
        } else if (deltaX > 0 and deltaY < 0) {
            alpha = 180 - alpha;
        }

    }
    return alpha;
}

bool Simu::isEatable(const RobotInfo &robot, const ParticleInfo &particle) {

    Angle alpha = angleBetween(robot.position, particle.position);

    Angle robotAngle = robot.angle;
    if (robotAngle < 0) {
        robotAngle += 360;
    } else if (robotAngle > 360) {
        robotAngle -= 360;
    }

    Angle beta = abs(robotAngle - alpha);

    if (beta > 360) {
        beta = 360 - beta;
    }

    if (robot.captureAngle >= beta) {
        return true;
    }

    return false;
}

template<>
RobotInfo Simu::getEntity<RobotInfo>(const State &state, size_t indexEntity) {
    return state.robots[indexEntity];
}

template<>
ParticleInfo Simu::getEntity<ParticleInfo>(const State &state, size_t indexEntity) {
    return state.particles[indexEntity];
}

template<typename T>
State Simu::dichotomicSearch(const State &initState, Time step, double stepValue,
                       size_t indexRobot, size_t indexEntity) {
    stepValue /= 2;

    State tempState = calculateNextState(initState, step + stepValue);
    RobotInfo tempRobot1 = tempState.robots[indexRobot];
    T tempEntity = getEntity<T>(tempState, indexEntity);

    double distanceBetweenEntities = distanceBetween(tempRobot1.position, tempEntity.position);

    if (abs(distanceBetweenEntities - tempRobot1.radius - tempEntity.radius) >
        EPSILON) {
        if (distanceBetweenEntities - tempRobot1.radius - tempEntity.radius < 0) {
            return dichotomicSearch<T>(initState, initState.time, stepValue, indexRobot,
                                       indexEntity);
        } else {
            return dichotomicSearch<T>(tempState, tempState.time, stepValue, indexRobot,
                                       indexEntity);
        }
    }

    tempState.robots[indexRobot].rightSpeed = 0;
    tempState.robots[indexRobot].leftSpeed = 0;

    if (is_same<T, RobotInfo>::value) {
        tempState.robots[indexEntity].rightSpeed = 0;
        tempState.robots[indexEntity].leftSpeed = 0;
    }

    return tempState;
}

bool Simu::explosionTimeIsGreater(const ParticleInfo &p1, const ParticleInfo &p2) {
    return p1.explosionTimes[0][0] > p2.explosionTimes[0][0];
}

bool Simu::calculateDirectionRobot(State &state, Time time) const {

    bool newStateDetected = false;

    bool isRotationCollision = false;

    state = calculateNextState(state, time);

    for (size_t i = 0; i < state.robots.size(); ++i) {

        RobotInfo &robot = state.robots[i];

        if (robot.idParticle == -1) {
            continue;
        }

        //If the robot is collisioning with another robot
//        if (robot.leftSpeed == 0 or robot.rightSpeed == 0){
//            Speed speedCollision;
//            if (isRotationCollision) {
//                speedCollision = min(constraints.maxBackwardSpeed, constraints.maxForwardSpeed);
//                robot.leftSpeed = -speedCollision;
//                robot.rightSpeed = speedCollision;
//                isRotationCollision = false;
//            } else {
//                speedCollision = constraints.maxBackwardSpeed;
//                robot.leftSpeed = robot.rightSpeed = -speedCollision;
//                isRotationCollision = true;
//            }
//            newStateDetected = true;
//            continue;
//        }

        ParticleInfo particleToEat;
        for (ParticleInfo &j: state.particles) {
            if (j.id == robot.idParticle) {
                particleToEat = j;
                break;
            }
        }

        Angle angleToParticule = angleBetween(robot.position,
                                              particleToEat.position);

        //Calculate the smallest angle between the robot and the particle
        Angle robotAngle = robot.angle;
        if (robotAngle < 0) {
            robotAngle += 360;
        } else if (robotAngle > 360) {
            robotAngle -= 360;
        }
        angleToParticule = angleToParticule - robotAngle;

        if (angleToParticule > 180) {
            angleToParticule -= 360;
        } else if (angleToParticule < -180) {
            angleToParticule += 360;
        }

        //Calculate the speed of the robot
        Speed speed;
        if (abs(angleToParticule) <= 5) {
            robot.leftSpeed = constraints.maxForwardSpeed;
            robot.rightSpeed = constraints.maxForwardSpeed;
        } else {
            Speed maxRotationSpeed = min(constraints.maxForwardSpeed, constraints.maxBackwardSpeed);
            Distance distanceWheel = 2 * M_PI * robot.radius * abs(angleToParticule) / 360;
            if (distanceWheel >= maxRotationSpeed * constraints.commandTimeInterval) {
                speed = maxRotationSpeed;
            } else {
                speed = distanceWheel / constraints.commandTimeInterval;
            }
            if (angleToParticule > 0) {
                robot.leftSpeed = speed;
                robot.rightSpeed = -speed;
            } else {
                robot.leftSpeed = -speed;
                robot.rightSpeed = speed;
            }
        }

        if (compareStateSpeed(state)) {
            newStateDetected = true;
        }

    }

    return newStateDetected;
}

bool Simu::compareStateSpeed(State const &state) const {
    State lastState = getLastState();
    for (size_t i = 0; i < state.robots.size(); ++i) {
        if (state.robots[i].leftSpeed != lastState.robots[i].leftSpeed or
            state.robots[i].rightSpeed != lastState.robots[i].rightSpeed) {
            return true;
        }
    }
    return false;
}

void
checkExtensionName(std::vector<std::string> const &ext, int argc, char *argv[]) {
    if (ext.size() != (argc - 1)) {
        cout << "Error number of arguements: mainSetup.cpp file.constraints initialState.stat timeline.tlin";
        cout << endl;
    }

    for (size_t i = 1; i <= ext.size(); ++i) {
        string s = argv[i];
        if (s.find(ext[i - 1]) == string::npos) {
            cout << "Error syntax: mainSetup.cpp file.constraints initialState.stat timeline.tlin" << endl;
            cout << "File type: " << ext[i - 1] << " not found" << endl;
            cout << endl;
            throw;
        }
    }
}

Constraints readConstraintsFile(string const &filename) {
    std::ifstream file(filename);

    if (!file.is_open()) {
        cout << "Error opening file constraints" << std::endl;
        throw;
    }

    Constraints constraints{};

    ifstream ifs(filename);
    ifs >> constraints;
    file.close();
    return constraints;
}

State readStateFile(string const &filename) {
    std::ifstream file(filename);

    if (!file.is_open()) {
        cout << "Error opening file initial state" << std::endl;
        throw;
    }

    ifstream ifs(filename);
    State state;
    ifs >> state;
    file.close();
    return state;
}

void exportTimelineFile(string const &filename, const Simu& simulation) {
    std::ofstream file(filename);

    if (!file.is_open()) {
        cout << "Error opening file timeline" << std::endl;
        throw;
    }

    ofstream ofs(filename);
    ofs << simulation.getTimeline();
    file.close();
}

bool compareDistance(const RobotInfo& a, const RobotInfo& b) {
    return a.Totaldistance > b.Totaldistance;
}

void Simu::particleNear(State &state) {
    vector<int> particleIsTaken;
    for (RobotInfo &robot: state.robots) {
        robot.idParticle = -1;
        for (ParticleInfo &particle: state.particles) {
            std::pair<double, int> particleDist;
            double distance = distanceBetween(robot.position, particle.position);
            particleDist.first = distance;
            particleDist.second = particle.id;
            robot.distanceParticle.push_back(particleDist);
            robot.Totaldistance += distance;
        }
    }

    std::sort(state.robots.begin(), state.robots.end(), compareDistance);

    for (RobotInfo &robot: state.robots) {
        if (robot.idParticle != -1) {
            continue;
        }
        for (int i = 0; i < state.particles.size(); ++i) {
            auto min = std::min_element(robot.distanceParticle.begin(), robot.distanceParticle.end());
            auto it = find(particleIsTaken.begin(), particleIsTaken.end(), min->second);
            if (it == particleIsTaken.end()) {
                particleIsTaken.push_back(min->second);
                robot.idParticle = min->second;
                break;
            } else {
                min->first = INT_MAX;
            }
        }
        robot.distanceParticle.clear();
    }
}
