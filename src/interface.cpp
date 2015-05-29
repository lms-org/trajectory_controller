#include "trajectory_point_follower.h"

extern "C" {

void* getInstance() {
    return new TrajectoryPointController();
}

}
