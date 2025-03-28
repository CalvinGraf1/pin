#include <string>
#include "Simu.h"

using namespace std;

int main(int argc, char *argv[]) {

    // Program arguments
    //	../../Data/Training/5_20_1_backward.constraints ../../Data/Training/training1.stat ../timeline.tlin

    checkExtensionName({".constraints", ".stat", ".tlin"}, argc, argv);

    string constraintFile = argv[1];
    string stateFile = argv[2];
    string timelineFile = argv[3];

    Simu simulation;

    simulation.setConstraints(readConstraintsFile(constraintFile));

    simulation.setFirstState(readStateFile(stateFile));

    simulation.calculateTimeline(simulation.getLastState());

    exportTimelineFile(timelineFile, simulation);

    return 0;
}
