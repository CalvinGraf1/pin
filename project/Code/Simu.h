#ifndef SIMU_H
#define SIMU_H

#include <utility>
#include <vector>
#include <list>
#include <string>
#include <cmath>
#include "Timeline.h"
#include "StateStream.h"
#include "Types.h"
#include "json_util.h"

struct Constraints {
    Speed commandTimeInterval;
    Speed maxBackwardSpeed;
    Speed maxForwardSpeed;

	NLOHMANN_DEFINE_TYPE_INTRUSIVE(Constraints, commandTimeInterval, maxBackwardSpeed,
                                   maxForwardSpeed)

	JSON_DEFINE_FLUX_OPERATORS(Constraints)
};

class Simu {
public:

	Simu() = default;

	void setFirstState(State const &state);

	void setConstraints(Constraints const &constraints);

	[[nodiscard]] State getLastState() const;

	[[nodiscard]] TimeLine getTimeline() const;

	void calculateTimeline(State state);

	bool calculateDirectionRobot(State &state, Time time) const;

private:

	const double EPSILON = 0.00001;
	double STEP_VALUE;

	std::vector<std::pair<RobotInfo, ParticleInfo>> robotEatParticle;

	TimeLine timeline;

	Constraints constraints{};

	void addState(const State &state);

	static bool verifyParticlesExplosion(State &state, double step, int &lastParticleId);

	bool verifyRobotsCollision(State &state, const State &previousState);

    static bool verifyRobotsCollisionSmall(State const &state) ;

	bool verifyRobotsParticlesCollision(State &state, const State &previousState);

	static std::vector<ParticleInfo> explodeParticle(const ParticleInfo &particleToExplode, int &lastParticleId);

	static Distance distanceBetween(const Position &pos1, const Position &pos2);

	bool eatParticle(State &state);

    static bool isEatable(const RobotInfo &robot, const ParticleInfo &particle);

	template<typename T>
	T getEntity(const State &state, size_t indexEntity);

	template<typename T>
	State dichotomicSearch(const State &initState, double step, double stepValue, size_t indexRobot, size_t indexEntity);

	static bool explosionTimeIsGreater(const ParticleInfo &p1, const ParticleInfo &p2);

	static void particleNear(State &state);

	[[nodiscard]] bool compareStateSpeed(State const &state) const;

	static Angle angleBetween(Position posRobot, Position posParticle) ;
};

void checkExtensionName(std::vector<std::string> const &ext, int argc, char *argv[]);

Constraints readConstraintsFile(std::string const &filename);

State readStateFile(std::string const &filename);

void exportTimelineFile(std::string const &filename, const Simu& simulation);

#endif //SIMU_H
