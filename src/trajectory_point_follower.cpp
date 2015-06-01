#include "trajectory_point_follower.h"
#include <cmath>
bool TrajectoryPointController::initialize() {
    controlData = datamanager()->writeChannel<Comm::SensorBoard::ControlData>(this,"CONTROL_DATA");
    trajectoryPoint = datamanager()->writeChannel<std::pair<lms::math::vertex2f,lms::math::vertex2f>>(this,"POINT");

    return true;
}

bool TrajectoryPointController::deinitialize() {
    return true;
}

bool TrajectoryPointController::cycle() {

    //std::pair<float,float> steering = TobisRegler_Simpel(trajectoryPoint->first.x, trajectoryPoint->first.y,
    //        atan2(trajectoryPoint->second.y, trajectoryPoint->second.x), 0.5);

    double phi_soll = atan2(trajectoryPoint->second.y, trajectoryPoint->second.x);
    double y_soll = trajectoryPoint->first.y;
    double x_soll = trajectoryPoint->first.x;
    controlData->control.velocity.velocity = 0.5*4;
    double v = controlData->control.velocity.velocity;
    logger.debug("soll: ")<< x_soll << " " << y_soll << " " << phi_soll;

    if(v <= 0)
        v = 0.05; //darf nicht 0 werden

    double t_end = ((fabs(x_soll) + fabs(y_soll)) + sqrt(x_soll * x_soll + y_soll * y_soll)) / (2.0 * v);

    //double delta_hinten = delta_h(phi_soll, t_end, v, y_soll);
    //double delta_vorne = delta_v(phi_soll, t_end, v, y_soll, delta_hinten);
    /*
    double delta_hinten = delta_h(y_soll, phi_soll, t_end);
    double delta_vorne = delta_v(y_soll, phi_soll, t_end, delta_hinten);
    */

    double delta_hinten = delta_c_h(phi_soll,t_end,v,y_soll);
    double delta_vorne = delta_c_v(phi_soll,t_end,v,y_soll,delta_hinten);
    logger.debug("cycle")<< "delta_hinten: \t" << delta_hinten ;
    logger.debug("cycle")<< "delta_vorne: \t" << delta_vorne ;
    logger.info("cycle")<< "------------------------------------";

    controlData->vel_mode = Comm::SensorBoard::ControlData::MODE_VELOCITY;
    controlData->steering_front = delta_vorne; // * 180. / M_PI;
    controlData->steering_rear = -delta_hinten; // * 180. / M_PI;
    //controlData->control.velocity.velocity = 0.5;
    return true;
}

/*std::pair<float,float> TrajectoryLineFollower::smartRegler(float x, float y, float phi){
    (void)x;
    logger.info("input_data: ") << y << " " << phi;
    float delta_v = 3*y;
    float delta_h = phi;
    return std::make_pair(delta_v, delta_h);
}*/

double TrajectoryPointController::delta_h(double y_s, double phi_s, double te)
{

    return (0.08577*exp(1.77*te)*exp(3.763*te)*(4.408*phi_s*exp(3.763*te) + 1.579*y_s*exp(3.763*te) + 1.153*phi_s*exp(1.77*te)*exp(7.526*te) - 5.56*phi_s*exp(3.541*te)*exp(3.763*te) - 3.572*y_s*exp(1.77*te)*exp(7.526*te) + 1.993*y_s*exp(3.541*te)*exp(3.763*te)))/(3.898*exp(5.311*te)*exp(11.29*te) - 6.203*exp(11.29*te)*exp(1.77*te) - 6.203*exp(5.311*te)*exp(3.763*te) + 3.898*exp(1.77*te)*exp(3.763*te) + 4.611*exp(3.541*te)*exp(7.526*te)) - (1.576*(0.5098*phi_s*exp(1.77*te) + 1.579*y_s*exp(1.77*te) + 0.6431*phi_s*exp(1.77*te)*exp(7.526*te) - 1.153*phi_s*exp(3.541*te)*exp(3.763*te) - 1.993*y_s*exp(1.77*te)*exp(7.526*te) + 0.4131*y_s*exp(3.541*te)*exp(3.763*te)))/(4.611*exp(1.77*te)*exp(3.763*te) - 6.203*exp(7.526*te) - 6.203*exp(3.541*te) + 3.898*exp(3.541*te)*exp(7.526*te) + 3.898) + (0.08577*exp(1.77*te)*exp(3.763*te)*(1.153*phi_s*exp(1.77*te) - 5.56*phi_s*exp(3.763*te) + 3.572*y_s*exp(1.77*te) - 1.993*y_s*exp(3.763*te) + 4.408*phi_s*exp(3.541*te)*exp(3.763*te) - 1.579*y_s*exp(3.541*te)*exp(3.763*te)))/(3.898*exp(5.311*te)*exp(11.29*te) - 6.203*exp(11.29*te)*exp(1.77*te) - 6.203*exp(5.311*te)*exp(3.763*te) + 3.898*exp(1.77*te)*exp(3.763*te) + 4.611*exp(3.541*te)*exp(7.526*te)) - (1.576*exp(1.77*te)*exp(3.763*te)*(0.6431*phi_s*exp(1.77*te) - 1.153*phi_s*exp(3.763*te) + 1.993*y_s*exp(1.77*te) - 0.4131*y_s*exp(3.763*te) + 0.5098*phi_s*exp(1.77*te)*exp(7.526*te) - 1.579*y_s*exp(1.77*te)*exp(7.526*te)))/(3.898*exp(5.311*te)*exp(11.29*te) - 6.203*exp(11.29*te)*exp(1.77*te) - 6.203*exp(5.311*te)*exp(3.763*te) + 3.898*exp(1.77*te)*exp(3.763*te) + 4.611*exp(3.541*te)*exp(7.526*te));

}

double TrajectoryPointController::delta_v(double y_s, double phi_s, double te, double dh)
{

    return atan((sin(dh) + (0.9206*(0.5098*phi_s*exp(1.77*te) + 1.579*y_s*exp(1.77*te) + 0.6431*phi_s*exp(1.77*te)*exp(7.526*te) - 1.153*phi_s*exp(3.541*te)*exp(3.763*te) - 1.993*y_s*exp(1.77*te)*exp(7.526*te) + 0.4131*y_s*exp(3.541*te)*exp(3.763*te)))/(4.611*exp(1.77*te)*exp(3.763*te) - 6.203*exp(7.526*te) - 6.203*exp(3.541*te) + 3.898*exp(3.541*te)*exp(7.526*te) + 3.898) - (1.957*exp(1.77*te)*exp(3.763*te)*(4.408*phi_s*exp(3.763*te) + 1.579*y_s*exp(3.763*te) + 1.153*phi_s*exp(1.77*te)*exp(7.526*te) - 5.56*phi_s*exp(3.541*te)*exp(3.763*te) - 3.572*y_s*exp(1.77*te)*exp(7.526*te) + 1.993*y_s*exp(3.541*te)*exp(3.763*te)))/(3.898*exp(5.311*te)*exp(11.29*te) - 6.203*exp(11.29*te)*exp(1.77*te) - 6.203*exp(5.311*te)*exp(3.763*te) + 3.898*exp(1.77*te)*exp(3.763*te) + 4.611*exp(3.541*te)*exp(7.526*te)) + (1.957*exp(1.77*te)*exp(3.763*te)*(1.153*phi_s*exp(1.77*te) - 5.56*phi_s*exp(3.763*te) + 3.572*y_s*exp(1.77*te) - 1.993*y_s*exp(3.763*te) + 4.408*phi_s*exp(3.541*te)*exp(3.763*te) - 1.579*y_s*exp(3.541*te)*exp(3.763*te)))/(3.898*exp(5.311*te)*exp(11.29*te) - 6.203*exp(11.29*te)*exp(1.77*te) - 6.203*exp(5.311*te)*exp(3.763*te) + 3.898*exp(1.77*te)*exp(3.763*te) + 4.611*exp(3.541*te)*exp(7.526*te)) - (0.9206*exp(1.77*te)*exp(3.763*te)*(0.6431*phi_s*exp(1.77*te) - 1.153*phi_s*exp(3.763*te) + 1.993*y_s*exp(1.77*te) - 0.4131*y_s*exp(3.763*te) + 0.5098*phi_s*exp(1.77*te)*exp(7.526*te) - 1.579*y_s*exp(1.77*te)*exp(7.526*te)))/(3.898*exp(5.311*te)*exp(11.29*te) - 6.203*exp(11.29*te)*exp(1.77*te) - 6.203*exp(5.311*te)*exp(3.763*te) + 3.898*exp(1.77*te)*exp(3.763*te) + 4.611*exp(3.541*te)*exp(7.526*te)))/cos(dh));

}

double TrajectoryPointController::delta_c_h(double phi_s, double te, double v, double y_s)
{
  return ((0.08577 * exp(te * sqrt(3.134 * (v * v))) * exp(te * sqrt(14.16 * (v *
              v))) * (((((((((4.984 * phi_s * (v * v) * exp(te * sqrt(14.16 * (v
    * v))) + 0.08653 * phi_s * exp(te * sqrt(14.16 * (v * v))) * sqrt(3.134 * (v
    * v)) * sqrt(14.16 * (v * v))) - 4.984 * phi_s * (v * v) * exp(te * sqrt
    (14.16 * (v * v))) * exp(2.0 * te * sqrt(3.134 * (v * v)))) - 2.018 * v *
    y_s * exp(te * sqrt(3.134 * (v * v))) * sqrt(3.134 * (v * v))) + 1.009 * v *
    y_s * exp(te * sqrt(14.16 * (v * v))) * sqrt(3.134 * (v * v))) + 0.0549 * v *
    y_s * exp(te * sqrt(14.16 * (v * v))) * sqrt(14.16 * (v * v))) - 0.1731 *
    phi_s * exp(te * sqrt(3.134 * (v * v))) * sqrt(3.134 * (v * v)) * sqrt(14.16
    * (v * v))) - 0.0549 * v * y_s * exp(te * sqrt(14.16 * (v * v))) * exp(2.0 *
    te * sqrt(3.134 * (v * v))) * sqrt(14.16 * (v * v))) + 0.08653 * phi_s * exp
                       (te * sqrt(14.16 * (v * v))) * exp(2.0 * te * sqrt(3.134 *
               (v * v))) * sqrt(3.134 * (v * v)) * sqrt(14.16 * (v * v))) +
                      1.009 * v * y_s * exp(te * sqrt(14.16 * (v * v))) * exp
                      (2.0 * te * sqrt(3.134 * (v * v))) * sqrt(3.134 * (v * v)))
           / ((((((((5.051 * (v * v) * exp(te * sqrt(3.134 * (v * v))) * exp(3.0
    * te * sqrt(14.16 * (v * v))) + 5.051 * (v * v) * exp(te * sqrt(14.16 * (v *
    v))) * exp(3.0 * te * sqrt(3.134 * (v * v)))) - 5.051 * (v * v) * exp(3.0 *
    te * sqrt(3.134 * (v * v))) * exp(3.0 * te * sqrt(14.16 * (v * v)))) - 5.051
                   * (v * v) * exp(te * sqrt(3.134 * (v * v))) * exp(te * sqrt
    (14.16 * (v * v)))) + 0.1731 * exp(te * sqrt(3.134 * (v * v))) * exp(3.0 *
    te * sqrt(14.16 * (v * v))) * sqrt(3.134 * (v * v)) * sqrt(14.16 * (v * v)))
                 + 0.1731 * exp(te * sqrt(14.16 * (v * v))) * exp(3.0 * te *
    sqrt(3.134 * (v * v))) * sqrt(3.134 * (v * v)) * sqrt(14.16 * (v * v))) -
                0.6922 * exp(2.0 * te * sqrt(3.134 * (v * v))) * exp(2.0 * te *
    sqrt(14.16 * (v * v))) * sqrt(3.134 * (v * v)) * sqrt(14.16 * (v * v))) +
               0.1731 * exp(3.0 * te * sqrt(3.134 * (v * v))) * exp(3.0 * te *
              sqrt(14.16 * (v * v))) * sqrt(3.134 * (v * v)) * sqrt(14.16 * (v *
    v))) + 0.1731 * exp(te * sqrt(3.134 * (v * v))) * exp(te * sqrt(14.16 * (v *
    v))) * sqrt(3.134 * (v * v)) * sqrt(14.16 * (v * v))) - 1.576 *
           (((((((((0.06667 * phi_s * (v * v) * exp(te * sqrt(3.134 * (v * v)))
                    - 0.06667 * phi_s * (v * v) * exp(te * sqrt(3.134 * (v * v)))
                    * exp(2.0 * te * sqrt(14.16 * (v * v)))) - 1.009 * v * y_s *
                   exp(te * sqrt(3.134 * (v * v))) * sqrt(3.134 * (v * v))) +
                  0.0549 * v * y_s * exp(te * sqrt(3.134 * (v * v))) * sqrt
                  (14.16 * (v * v))) - 0.08653 * phi_s * exp(te * sqrt(3.134 *
    (v * v))) * sqrt(3.134 * (v * v)) * sqrt(14.16 * (v * v))) + 0.0549 * v *
                y_s * exp(te * sqrt(3.134 * (v * v))) * exp(2.0 * te * sqrt
    (14.16 * (v * v))) * sqrt(14.16 * (v * v))) - 0.1098 * v * y_s * exp(te *
    sqrt(14.16 * (v * v))) * exp(2.0 * te * sqrt(3.134 * (v * v))) * sqrt(14.16 *
                (v * v))) - 0.08653 * phi_s * exp(te * sqrt(3.134 * (v * v))) *
              exp(2.0 * te * sqrt(14.16 * (v * v))) * sqrt(3.134 * (v * v)) *
              sqrt(14.16 * (v * v))) + 0.1731 * phi_s * exp(te * sqrt(14.16 * (v
    * v))) * exp(2.0 * te * sqrt(3.134 * (v * v))) * sqrt(3.134 * (v * v)) *
             sqrt(14.16 * (v * v))) + 1.009 * v * y_s * exp(te * sqrt(3.134 * (v
    * v))) * exp(2.0 * te * sqrt(14.16 * (v * v))) * sqrt(3.134 * (v * v))) /
           ((((((((0.1731 * sqrt(3.134 * (v * v)) * sqrt(14.16 * (v * v)) +
                   5.051 * (v * v) * exp(2.0 * te * sqrt(3.134 * (v * v)))) +
                  5.051 * (v * v) * exp(2.0 * te * sqrt(14.16 * (v * v)))) -
                 5.051 * (v * v)) - 5.051 * (v * v) * exp(2.0 * te * sqrt(3.134 *
                  (v * v))) * exp(2.0 * te * sqrt(14.16 * (v * v)))) + 0.1731 *
               exp(2.0 * te * sqrt(3.134 * (v * v))) * sqrt(3.134 * (v * v)) *
               sqrt(14.16 * (v * v))) + 0.1731 * exp(2.0 * te * sqrt(14.16 * (v *
    v))) * sqrt(3.134 * (v * v)) * sqrt(14.16 * (v * v))) + 0.1731 * exp(2.0 *
              te * sqrt(3.134 * (v * v))) * exp(2.0 * te * sqrt(14.16 * (v * v)))
             * sqrt(3.134 * (v * v)) * sqrt(14.16 * (v * v))) - 0.6922 * exp(te *
             sqrt(3.134 * (v * v))) * exp(te * sqrt(14.16 * (v * v))) * sqrt
            (3.134 * (v * v)) * sqrt(14.16 * (v * v)))) + 1.576 * exp(te * sqrt
           (3.134 * (v * v))) * exp(te * sqrt(14.16 * (v * v))) *
          (((((((((0.06667 * phi_s * (v * v) * exp(te * sqrt(3.134 * (v * v))) -
                   0.1731 * phi_s * exp(te * sqrt(14.16 * (v * v))) * sqrt(3.134
    * (v * v)) * sqrt(14.16 * (v * v))) - 0.06667 * phi_s * (v * v) * exp(te *
    sqrt(3.134 * (v * v))) * exp(2.0 * te * sqrt(14.16 * (v * v)))) + 1.009 * v *
                 y_s * exp(te * sqrt(3.134 * (v * v))) * sqrt(3.134 * (v * v)))
                + 0.0549 * v * y_s * exp(te * sqrt(3.134 * (v * v))) * sqrt
                (14.16 * (v * v))) - 0.1098 * v * y_s * exp(te * sqrt(14.16 * (v
    * v))) * sqrt(14.16 * (v * v))) + 0.08653 * phi_s * exp(te * sqrt(3.134 * (v
    * v))) * sqrt(3.134 * (v * v)) * sqrt(14.16 * (v * v))) + 0.0549 * v * y_s *
             exp(te * sqrt(3.134 * (v * v))) * exp(2.0 * te * sqrt(14.16 * (v *
    v))) * sqrt(14.16 * (v * v))) + 0.08653 * phi_s * exp(te * sqrt(3.134 * (v *
    v))) * exp(2.0 * te * sqrt(14.16 * (v * v))) * sqrt(3.134 * (v * v)) * sqrt
            (14.16 * (v * v))) - 1.009 * v * y_s * exp(te * sqrt(3.134 * (v * v)))
           * exp(2.0 * te * sqrt(14.16 * (v * v))) * sqrt(3.134 * (v * v))) /
          ((((((((5.051 * (v * v) * exp(te * sqrt(3.134 * (v * v))) * exp(3.0 *
    te * sqrt(14.16 * (v * v))) + 5.051 * (v * v) * exp(te * sqrt(14.16 * (v * v)))
                  * exp(3.0 * te * sqrt(3.134 * (v * v)))) - 5.051 * (v * v) *
                 exp(3.0 * te * sqrt(3.134 * (v * v))) * exp(3.0 * te * sqrt
    (14.16 * (v * v)))) - 5.051 * (v * v) * exp(te * sqrt(3.134 * (v * v))) *
                exp(te * sqrt(14.16 * (v * v)))) + 0.1731 * exp(te * sqrt(3.134 *
                 (v * v))) * exp(3.0 * te * sqrt(14.16 * (v * v))) * sqrt(3.134 *
                (v * v)) * sqrt(14.16 * (v * v))) + 0.1731 * exp(te * sqrt(14.16
    * (v * v))) * exp(3.0 * te * sqrt(3.134 * (v * v))) * sqrt(3.134 * (v * v)) *
              sqrt(14.16 * (v * v))) - 0.6922 * exp(2.0 * te * sqrt(3.134 * (v *
    v))) * exp(2.0 * te * sqrt(14.16 * (v * v))) * sqrt(3.134 * (v * v)) * sqrt
             (14.16 * (v * v))) + 0.1731 * exp(3.0 * te * sqrt(3.134 * (v * v)))
            * exp(3.0 * te * sqrt(14.16 * (v * v))) * sqrt(3.134 * (v * v)) *
            sqrt(14.16 * (v * v))) + 0.1731 * exp(te * sqrt(3.134 * (v * v))) *
           exp(te * sqrt(14.16 * (v * v))) * sqrt(3.134 * (v * v)) * sqrt(14.16 *
            (v * v)))) - 0.08577 * exp(te * sqrt(3.134 * (v * v))) * exp(te *
    sqrt(14.16 * (v * v))) * (((((((((4.984 * phi_s * (v * v) * exp(te * sqrt
    (14.16 * (v * v))) - 0.08653 * phi_s * exp(te * sqrt(14.16 * (v * v))) *
    sqrt(3.134 * (v * v)) * sqrt(14.16 * (v * v))) - 4.984 * phi_s * (v * v) *
    exp(te * sqrt(14.16 * (v * v))) * exp(2.0 * te * sqrt(3.134 * (v * v)))) +
    1.009 * v * y_s * exp(te * sqrt(14.16 * (v * v))) * sqrt(3.134 * (v * v))) -
    0.0549 * v * y_s * exp(te * sqrt(14.16 * (v * v))) * sqrt(14.16 * (v * v)))
    + 0.0549 * v * y_s * exp(te * sqrt(14.16 * (v * v))) * exp(2.0 * te * sqrt
    (3.134 * (v * v))) * sqrt(14.16 * (v * v))) + 0.1731 * phi_s * exp(te * sqrt
                                  (3.134 * (v * v))) * exp(2.0 * te * sqrt(14.16
    * (v * v))) * sqrt(3.134 * (v * v)) * sqrt(14.16 * (v * v))) - 0.08653 *
    phi_s * exp(te * sqrt(14.16 * (v * v))) * exp(2.0 * te * sqrt(3.134 * (v * v)))
    * sqrt(3.134 * (v * v)) * sqrt(14.16 * (v * v))) - 2.018 * v * y_s * exp(te *
    sqrt(3.134 * (v * v))) * exp(2.0 * te * sqrt(14.16 * (v * v))) * sqrt(3.134 *
                                (v * v))) + 1.009 * v * y_s * exp(te * sqrt
    (14.16 * (v * v))) * exp(2.0 * te * sqrt(3.134 * (v * v))) * sqrt(3.134 * (v
    * v))) / ((((((((5.051 * (v * v) * exp(te * sqrt(3.134 * (v * v))) * exp(3.0
    * te * sqrt(14.16 * (v * v))) + 5.051 * (v * v) * exp(te * sqrt(14.16 * (v *
    v))) * exp(3.0 * te * sqrt(3.134 * (v * v)))) - 5.051 * (v * v) * exp(3.0 *
    te * sqrt(3.134 * (v * v))) * exp(3.0 * te * sqrt(14.16 * (v * v)))) - 5.051
                   * (v * v) * exp(te * sqrt(3.134 * (v * v))) * exp(te * sqrt
    (14.16 * (v * v)))) + 0.1731 * exp(te * sqrt(3.134 * (v * v))) * exp(3.0 *
    te * sqrt(14.16 * (v * v))) * sqrt(3.134 * (v * v)) * sqrt(14.16 * (v * v)))
                 + 0.1731 * exp(te * sqrt(14.16 * (v * v))) * exp(3.0 * te *
    sqrt(3.134 * (v * v))) * sqrt(3.134 * (v * v)) * sqrt(14.16 * (v * v))) -
                0.6922 * exp(2.0 * te * sqrt(3.134 * (v * v))) * exp(2.0 * te *
    sqrt(14.16 * (v * v))) * sqrt(3.134 * (v * v)) * sqrt(14.16 * (v * v))) +
               0.1731 * exp(3.0 * te * sqrt(3.134 * (v * v))) * exp(3.0 * te *
    sqrt(14.16 * (v * v))) * sqrt(3.134 * (v * v)) * sqrt(14.16 * (v * v))) +
              0.1731 * exp(te * sqrt(3.134 * (v * v))) * exp(te * sqrt(14.16 *
    (v * v))) * sqrt(3.134 * (v * v)) * sqrt(14.16 * (v * v)));
}

double TrajectoryPointController::delta_c_v(double phi_s, double te, double v, double y_s, double dh)
{
  return atan((sin(dh) + 0.52 * (((sqrt(3.134 * (v * v)) * (((((((((0.06667 *
    phi_s * (v * v) * exp(te * sqrt(3.134 * (v * v))) - 0.06667 * phi_s * (v * v)
    * exp(te * sqrt(3.134 * (v * v))) * exp(2.0 * te * sqrt(14.16 * (v * v)))) -
    1.009 * v * y_s * exp(te * sqrt(3.134 * (v * v))) * sqrt(3.134 * (v * v))) +
    0.0549 * v * y_s * exp(te * sqrt(3.134 * (v * v))) * sqrt(14.16 * (v * v)))
    - 0.08653 * phi_s * exp(te * sqrt(3.134 * (v * v))) * sqrt(3.134 * (v * v)) *
    sqrt(14.16 * (v * v))) + 0.0549 * v * y_s * exp(te * sqrt(3.134 * (v * v))) *
    exp(2.0 * te * sqrt(14.16 * (v * v))) * sqrt(14.16 * (v * v))) - 0.1098 * v *
    y_s * exp(te * sqrt(14.16 * (v * v))) * exp(2.0 * te * sqrt(3.134 * (v * v)))
    * sqrt(14.16 * (v * v))) - 0.08653 * phi_s * exp(te * sqrt(3.134 * (v * v)))
    * exp(2.0 * te * sqrt(14.16 * (v * v))) * sqrt(3.134 * (v * v)) * sqrt(14.16
    * (v * v))) + 0.1731 * phi_s * exp(te * sqrt(14.16 * (v * v))) * exp(2.0 *
    te * sqrt(3.134 * (v * v))) * sqrt(3.134 * (v * v)) * sqrt(14.16 * (v * v)))
    + 1.009 * v * y_s * exp(te * sqrt(3.134 * (v * v))) * exp(2.0 * te * sqrt
    (14.16 * (v * v))) * sqrt(3.134 * (v * v))) / ((((((((0.1731 * sqrt(3.134 *
                           (v * v)) * sqrt(14.16 * (v * v)) + 5.051 * (v * v) *
    exp(2.0 * te * sqrt(3.134 * (v * v)))) + 5.051 * (v * v) * exp(2.0 * te *
    sqrt(14.16 * (v * v)))) - 5.051 * (v * v)) - 5.051 * (v * v) * exp(2.0 * te *
    sqrt(3.134 * (v * v))) * exp(2.0 * te * sqrt(14.16 * (v * v)))) + 0.1731 *
    exp(2.0 * te * sqrt(3.134 * (v * v))) * sqrt(3.134 * (v * v)) * sqrt(14.16 *
                       (v * v))) + 0.1731 * exp(2.0 * te * sqrt(14.16 * (v * v)))
    * sqrt(3.134 * (v * v)) * sqrt(14.16 * (v * v))) + 0.1731 * exp(2.0 * te *
    sqrt(3.134 * (v * v))) * exp(2.0 * te * sqrt(14.16 * (v * v))) * sqrt(3.134 *
                     (v * v)) * sqrt(14.16 * (v * v))) - 0.6922 * exp(te * sqrt
    (3.134 * (v * v))) * exp(te * sqrt(14.16 * (v * v))) * sqrt(3.134 * (v * v))
    * sqrt(14.16 * (v * v))) + exp(te * sqrt(3.134 * (v * v))) * exp(te * sqrt
    (14.16 * (v * v))) * sqrt(14.16 * (v * v)) * (((((((((4.984 * phi_s * (v * v)
    * exp(te * sqrt(14.16 * (v * v))) + 0.08653 * phi_s * exp(te * sqrt(14.16 *
                             (v * v))) * sqrt(3.134 * (v * v)) * sqrt(14.16 * (v
    * v))) - 4.984 * phi_s * (v * v) * exp(te * sqrt(14.16 * (v * v))) * exp(2.0
    * te * sqrt(3.134 * (v * v)))) - 2.018 * v * y_s * exp(te * sqrt(3.134 * (v *
    v))) * sqrt(3.134 * (v * v))) + 1.009 * v * y_s * exp(te * sqrt(14.16 * (v *
    v))) * sqrt(3.134 * (v * v))) + 0.0549 * v * y_s * exp(te * sqrt(14.16 * (v *
    v))) * sqrt(14.16 * (v * v))) - 0.1731 * phi_s * exp(te * sqrt(3.134 * (v *
    v))) * sqrt(3.134 * (v * v)) * sqrt(14.16 * (v * v))) - 0.0549 * v * y_s *
    exp(te * sqrt(14.16 * (v * v))) * exp(2.0 * te * sqrt(3.134 * (v * v))) *
    sqrt(14.16 * (v * v))) + 0.08653 * phi_s * exp(te * sqrt(14.16 * (v * v))) *
    exp(2.0 * te * sqrt(3.134 * (v * v))) * sqrt(3.134 * (v * v)) * sqrt(14.16 *
                     (v * v))) + 1.009 * v * y_s * exp(te * sqrt(14.16 * (v * v)))
    * exp(2.0 * te * sqrt(3.134 * (v * v))) * sqrt(3.134 * (v * v))) /
    ((((((((5.051 * (v * v) * exp(te * sqrt(3.134 * (v * v))) * exp(3.0 * te *
    sqrt(14.16 * (v * v))) + 5.051 * (v * v) * exp(te * sqrt(14.16 * (v * v))) *
            exp(3.0 * te * sqrt(3.134 * (v * v)))) - 5.051 * (v * v) * exp(3.0 *
    te * sqrt(3.134 * (v * v))) * exp(3.0 * te * sqrt(14.16 * (v * v)))) - 5.051
          * (v * v) * exp(te * sqrt(3.134 * (v * v))) * exp(te * sqrt(14.16 * (v
    * v)))) + 0.1731 * exp(te * sqrt(3.134 * (v * v))) * exp(3.0 * te * sqrt
    (14.16 * (v * v))) * sqrt(3.134 * (v * v)) * sqrt(14.16 * (v * v))) + 0.1731
        * exp(te * sqrt(14.16 * (v * v))) * exp(3.0 * te * sqrt(3.134 * (v * v)))
        * sqrt(3.134 * (v * v)) * sqrt(14.16 * (v * v))) - 0.6922 * exp(2.0 * te
    * sqrt(3.134 * (v * v))) * exp(2.0 * te * sqrt(14.16 * (v * v))) * sqrt
       (3.134 * (v * v)) * sqrt(14.16 * (v * v))) + 0.1731 * exp(3.0 * te * sqrt
                     (3.134 * (v * v))) * exp(3.0 * te * sqrt(14.16 * (v * v))) *
      sqrt(3.134 * (v * v)) * sqrt(14.16 * (v * v))) + 0.1731 * exp(te * sqrt
    (3.134 * (v * v))) * exp(te * sqrt(14.16 * (v * v))) * sqrt(3.134 * (v * v))
     * sqrt(14.16 * (v * v)))) + exp(te * sqrt(3.134 * (v * v))) * exp(te * sqrt
                  (14.16 * (v * v))) * sqrt(3.134 * (v * v)) * (((((((((0.06667 *
    phi_s * (v * v) * exp(te * sqrt(3.134 * (v * v))) - 0.1731 * phi_s * exp(te *
    sqrt(14.16 * (v * v))) * sqrt(3.134 * (v * v)) * sqrt(14.16 * (v * v))) -
    0.06667 * phi_s * (v * v) * exp(te * sqrt(3.134 * (v * v))) * exp(2.0 * te *
    sqrt(14.16 * (v * v)))) + 1.009 * v * y_s * exp(te * sqrt(3.134 * (v * v))) *
    sqrt(3.134 * (v * v))) + 0.0549 * v * y_s * exp(te * sqrt(3.134 * (v * v))) *
    sqrt(14.16 * (v * v))) - 0.1098 * v * y_s * exp(te * sqrt(14.16 * (v * v))) *
    sqrt(14.16 * (v * v))) + 0.08653 * phi_s * exp(te * sqrt(3.134 * (v * v))) *
    sqrt(3.134 * (v * v)) * sqrt(14.16 * (v * v))) + 0.0549 * v * y_s * exp(te *
    sqrt(3.134 * (v * v))) * exp(2.0 * te * sqrt(14.16 * (v * v))) * sqrt(14.16 *
                     (v * v))) + 0.08653 * phi_s * exp(te * sqrt(3.134 * (v * v)))
    * exp(2.0 * te * sqrt(14.16 * (v * v))) * sqrt(3.134 * (v * v)) * sqrt(14.16
    * (v * v))) - 1.009 * v * y_s * exp(te * sqrt(3.134 * (v * v))) * exp(2.0 *
    te * sqrt(14.16 * (v * v))) * sqrt(3.134 * (v * v))) / ((((((((5.051 * (v *
    v) * exp(te * sqrt(3.134 * (v * v))) * exp(3.0 * te * sqrt(14.16 * (v * v)))
    + 5.051 * (v * v) * exp(te * sqrt(14.16 * (v * v))) * exp(3.0 * te * sqrt
    (3.134 * (v * v)))) - 5.051 * (v * v) * exp(3.0 * te * sqrt(3.134 * (v * v)))
    * exp(3.0 * te * sqrt(14.16 * (v * v)))) - 5.051 * (v * v) * exp(te * sqrt
    (3.134 * (v * v))) * exp(te * sqrt(14.16 * (v * v)))) + 0.1731 * exp(te *
    sqrt(3.134 * (v * v))) * exp(3.0 * te * sqrt(14.16 * (v * v))) * sqrt(3.134 *
                       (v * v)) * sqrt(14.16 * (v * v))) + 0.1731 * exp(te *
    sqrt(14.16 * (v * v))) * exp(3.0 * te * sqrt(3.134 * (v * v))) * sqrt(3.134 *
                      (v * v)) * sqrt(14.16 * (v * v))) - 0.6922 * exp(2.0 * te *
    sqrt(3.134 * (v * v))) * exp(2.0 * te * sqrt(14.16 * (v * v))) * sqrt(3.134 *
                     (v * v)) * sqrt(14.16 * (v * v))) + 0.1731 * exp(3.0 * te *
    sqrt(3.134 * (v * v))) * exp(3.0 * te * sqrt(14.16 * (v * v))) * sqrt(3.134 *
                    (v * v)) * sqrt(14.16 * (v * v))) + 0.1731 * exp(te * sqrt
    (3.134 * (v * v))) * exp(te * sqrt(14.16 * (v * v))) * sqrt(3.134 * (v * v))
    * sqrt(14.16 * (v * v)))) + exp(te * sqrt(3.134 * (v * v))) * exp(te * sqrt
    (14.16 * (v * v))) * sqrt(14.16 * (v * v)) * (((((((((4.984 * phi_s * (v * v)
    * exp(te * sqrt(14.16 * (v * v))) - 0.08653 * phi_s * exp(te * sqrt(14.16 *
                           (v * v))) * sqrt(3.134 * (v * v)) * sqrt(14.16 * (v *
    v))) - 4.984 * phi_s * (v * v) * exp(te * sqrt(14.16 * (v * v))) * exp(2.0 *
    te * sqrt(3.134 * (v * v)))) + 1.009 * v * y_s * exp(te * sqrt(14.16 * (v *
    v))) * sqrt(3.134 * (v * v))) - 0.0549 * v * y_s * exp(te * sqrt(14.16 * (v *
    v))) * sqrt(14.16 * (v * v))) + 0.0549 * v * y_s * exp(te * sqrt(14.16 * (v *
    v))) * exp(2.0 * te * sqrt(3.134 * (v * v))) * sqrt(14.16 * (v * v))) +
    0.1731 * phi_s * exp(te * sqrt(3.134 * (v * v))) * exp(2.0 * te * sqrt(14.16
    * (v * v))) * sqrt(3.134 * (v * v)) * sqrt(14.16 * (v * v))) - 0.08653 *
    phi_s * exp(te * sqrt(14.16 * (v * v))) * exp(2.0 * te * sqrt(3.134 * (v * v)))
    * sqrt(3.134 * (v * v)) * sqrt(14.16 * (v * v))) - 2.018 * v * y_s * exp(te *
    sqrt(3.134 * (v * v))) * exp(2.0 * te * sqrt(14.16 * (v * v))) * sqrt(3.134 *
                   (v * v))) + 1.009 * v * y_s * exp(te * sqrt(14.16 * (v * v)))
    * exp(2.0 * te * sqrt(3.134 * (v * v))) * sqrt(3.134 * (v * v))) /
    ((((((((5.051 * (v * v) * exp(te * sqrt(3.134 * (v * v))) * exp(3.0 * te *
    sqrt(14.16 * (v * v))) + 5.051 * (v * v) * exp(te * sqrt(14.16 * (v * v))) *
            exp(3.0 * te * sqrt(3.134 * (v * v)))) - 5.051 * (v * v) * exp(3.0 *
    te * sqrt(3.134 * (v * v))) * exp(3.0 * te * sqrt(14.16 * (v * v)))) - 5.051
          * (v * v) * exp(te * sqrt(3.134 * (v * v))) * exp(te * sqrt(14.16 * (v
    * v)))) + 0.1731 * exp(te * sqrt(3.134 * (v * v))) * exp(3.0 * te * sqrt
    (14.16 * (v * v))) * sqrt(3.134 * (v * v)) * sqrt(14.16 * (v * v))) + 0.1731
        * exp(te * sqrt(14.16 * (v * v))) * exp(3.0 * te * sqrt(3.134 * (v * v)))
        * sqrt(3.134 * (v * v)) * sqrt(14.16 * (v * v))) - 0.6922 * exp(2.0 * te
    * sqrt(3.134 * (v * v))) * exp(2.0 * te * sqrt(14.16 * (v * v))) * sqrt
       (3.134 * (v * v)) * sqrt(14.16 * (v * v))) + 0.1731 * exp(3.0 * te * sqrt
                   (3.134 * (v * v))) * exp(3.0 * te * sqrt(14.16 * (v * v))) *
      sqrt(3.134 * (v * v)) * sqrt(14.16 * (v * v))) + 0.1731 * exp(te * sqrt
    (3.134 * (v * v))) * exp(te * sqrt(14.16 * (v * v))) * sqrt(3.134 * (v * v))
     * sqrt(14.16 * (v * v)))) / v) / cos(dh));
}





