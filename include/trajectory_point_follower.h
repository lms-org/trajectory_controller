#ifndef IMAGE_HINT_TRANSFORMER_H
#define IMAGE_HINT_TRANSFORMER_H

#include "lms/module.h"
#include "lms/math/vertex.h"
#include "lms/math/lookup_table.h"

#include "sensor_utils/car.h"
#include "street_environment/trajectory.h"
#include "lms/math/interpolation.h"
#include "lms/math/lookup_table.h"
#include "sensor_utils/pid_controller.h"
#include "phoenix_CC2016_service/phoenix_CC2016_service.h"

//http://stackoverflow.com/questions/7159348/disable-single-warning-error
//http://stackoverflow.com/questions/3378560/how-to-disable-gcc-warnings-for-a-few-lines-of-code
//#pragma warning (disable : 4068 ) /* disable unknown pragma warnings */
#pragma GCC diagnostic ignored "-Wpedantic"
#include <dlib/control.h>         /* no diagnostic for this one */
#pragma GCC diagnostic pop

#include <ctime>
#include <unistd.h>
#include <fstream>

class TrajectoryPointController : public lms::Module {
public:
    bool initialize() override;
    bool deinitialize() override;
    bool cycle() override;
private:

    lms::math::LookupTable<float, lms::math::LookupTableOrder::ASC> m_mpcLookupVelocity;
    lms::math::LookupTable<float,lms::math::LookupTableOrder::ASC> m_trajectoryPointDistanceLookup;

    street_environment::TrajectoryPoint getTrajectoryPoint(const float distanceToPoint);
    /**
     * @brief slowDownCar used to slow down the car if the trajectory comes to an end
     */
    sensor_utils::PID slowDownCar;

    void mpcController(double v, double delta_y, double delta_phi, double *steering_front, double *steering_rear);
    void configsChanged() override;

    double l = 0.21; //Radstand
    static constexpr size_t MPC_HORIZON = 4;

    struct MpcParameters
    {
        double weight_y, weight_phi, weight_steeringFront, weight_steeringRear, stepSize;
    } mpcParameters;

    std::ofstream myfile;
    double v_global;
    dlib::matrix<double,2,1> lower, upper;

    lms::ReadDataChannel<street_environment::Trajectory> trajectory;
    lms::WriteDataChannel<street_environment::Car> car;
    /**
     * @brief trajectoryPoint only for debugging
     */
    lms::WriteDataChannel<street_environment::TrajectoryPoint> debugging_trajectoryPoint;

    lms::Time turnStart;
    bool isTurn;

};

#endif /* IMAGE_HINT_TRANSFORMER_H */
