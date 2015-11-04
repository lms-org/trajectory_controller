#include "trajectory_point_follower.h"
extern "C"{
#include "lenkwinkel.h"
}
#include <cmath>
bool TrajectoryPointController::initialize() {
    config = getConfig();
    trajectoryPoint = datamanager()->writeChannel<std::pair<lms::math::vertex2f,lms::math::vertex2f>>(this,"POINT");
    car = datamanager()->writeChannel<sensor_utils::Car>(this,"CAR");
    return true;
}

bool TrajectoryPointController::deinitialize() {
    return true;
}

bool TrajectoryPointController::cycle() {
    positionController();
    return true;
}


void TrajectoryPointController::positionController(){
    double phi_soll = atan2(trajectoryPoint->second.y, trajectoryPoint->second.x);
    double y_soll = trajectoryPoint->first.y;
    double x_soll = trajectoryPoint->first.x;

    double delta_hinten;
    double delta_vorne;

    lenkwinkel(x_soll, y_soll, phi_soll,config->get<int>("regler",1), &delta_hinten,
      &delta_vorne);

    logger.debug("positionController")<<delta_vorne<<" "<<delta_hinten;
    if(isnan(delta_vorne) || isnan(delta_hinten) ){
        logger.error("positionController: ")<<"invalid vals: " <<delta_vorne <<" " <<delta_hinten ;
        delta_vorne = 0;
        delta_hinten = 0;
    }
    //set the default state
    sensor_utils::Car::State state;
    state.name = "DEFAULT";
    state.steering_front = delta_vorne; // * 180. / M_PI;
    state.steering_rear = delta_hinten; // * 180. / M_PI;
    car->putState(state);
}





