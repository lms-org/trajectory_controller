#include "trajectory_point_follower.h"
extern "C"{
#include "lenkwinkel.h"
}
#include <cmath>



bool TrajectoryPointController::initialize() {
    config = getConfig();
    trajectoryPoint = datamanager()->readChannel<std::pair<lms::math::vertex2f,lms::math::vertex2f>>(this,"POINT");
    car = datamanager()->writeChannel<sensor_utils::Car>(this,"CAR");

    //Stellgroessenbeschraenkung
    double alpha_max = 32*M_PI/180;
    lower = -alpha_max, -alpha_max;
    upper =  alpha_max,  alpha_max;

    return true;
}

bool TrajectoryPointController::deinitialize() {
    return true;
}

bool TrajectoryPointController::cycle() {

    //TODO von config einlesen, um live einzustellen
    mpcParameters.weight_y = 3;
    mpcParameters.weight_phi = 3;
    mpcParameters.weight_steeringFront = 1;
    mpcParameters.weight_steeringRear = 1;

    double T = 0.1; //Zeitschrittgroesse fuer MPC

    //Simulation
    /*double v = 1;
    double xf = 0.5;
    double y0 = 1;
    double lambda = xf/cos(phi) + x - tan(phi)*(y0-y);
    double delta_y = x*sin(phi) + cos(phi)*(y0-y)-lambda*sin(phi);
    double delta_phi = -phi;*/

    double phi_soll = atan2(trajectoryPoint->second.y, trajectoryPoint->second.x);
    double y_soll = trajectoryPoint->first.y;
    //double v = sensor_utils::Car::velocity();
    double v = 1;

    double steering_front, steering_rear;
    mpcController(T, v, y_soll, phi_soll, &steering_front, &steering_rear);


    // Simulation
    /*std::cout << steering_front <<";" << steering_rear << "\n";
    x = x + T*v*cos(steering_rear + phi);
    y = y + T*v*sin(steering_rear + phi);
    phi = phi + T*v/0.21*sin(steering_front - steering_rear)/cos(steering_rear);*/


    // Zeitmessung
    /*clock_t start = clock();
    for(int i=0; i < 1000; ++i) {
        mpcController(T, v, y_soll, phi_soll, &steering_front, &steering_rear);
    }
    logger.debug("trajectory_point_controller") << "elapsed time: " << (double)(clock()-start)/CLOCKS_PER_SEC*1000;*/


    logger.debug("trajectory_point_controller") << "lw vorne: " << steering_front << "  lw hinten: " << steering_rear;
    if(isnan(steering_front) || isnan(steering_rear) ){
        logger.error("trajectory_point_controller: ")<<"invalid vals: " <<steering_front <<" " <<steering_rear ;
    }

    //set the default state
    sensor_utils::Car::State state;
    state.name = "DEFAULT";
    state.steering_front = steering_front; // * 180. / M_PI;
    state.steering_rear = steering_rear; // * 180. / M_PI;
    car->putState(state);

    return true;
}

void TrajectoryPointController::mpcController(double T, double v, double delta_y, double delta_phi, double *steering_front, double *steering_rear) {

    const int STATES = 2;
    const int CONTROLS = 2;

    // Modell festlegen
    // x_{i+1} == A*x_i + B*u_i + C

    dlib::matrix<double,STATES,STATES> A;
    A = 1, T*v, 0, 1;

    dlib::matrix<double,STATES,CONTROLS> B;
    B = 0, T*v, T*v/l, -T*v/l;

    dlib::matrix<double,STATES,1> C; //keine konstante Stoerung
    C = 0, 0;

    // Now we need to setup some MPC specific parameters.  To understand them,
    // let's first talk about how MPC works.  When the MPC tool finds the "best"
    // control to apply it does it by simulating the process for HORIZON time
    // steps and selecting the control that leads to the best performance over
    // the next HORIZON steps.
    //
    // To be precise, each time you ask it for a control, it solves the
    // following quadratic program:
    //
    //     min     sum_i trans(x_i-target_i)*Q*(x_i-target_i) + trans(u_i)*R*u_i
    //    x_i,u_i
    //
    //     such that: x_0     == current_state
    //                x_{i+1} == A*x_i + B*u_i + C
    //                lower <= u_i <= upper
    //                0 <= i < HORIZON
    //
    // and reports u_0 as the control you should take given that you are currently
    // in current_state.  Q and R are user supplied matrices that define how we
    // penalize variations away from the target state as well as how much we want
    // to avoid generating large control signals.  We also allow you to specify
    // upper and lower bound constraints on the controls.  The next few lines
    // define these parameters for our simple example.

    dlib::matrix<double,STATES,1> Q; //Gewichtung der Regelabweichung fuer Querabstand y und Orientierung phi
    Q = mpcParameters.weight_y, mpcParameters.weight_phi;

    dlib::matrix<double,STATES,1> R; //Gewichtung der Stellgroessen
    R = mpcParameters.weight_steeringFront, mpcParameters.weight_steeringRear;


    dlib::mpc<STATES,CONTROLS,mpcParameters.HORIZON> controller(A,B,C,Q,R,lower,upper); //30*T ist der Zeithorizont fuer die praediktion, d.h. 30 Zeitschritte wird in die Zukunft simuliert

    dlib::matrix<double,STATES,1> target;
    target = delta_y, delta_phi;

    controller.set_target(target);

    //Stellschrauben fuer performance
    //controller.set_epsilon(0.05);
    //controller.set_max_iterations(300);

    dlib::matrix<double,STATES,1> current_state = {0,0};

    dlib::matrix<double,CONTROLS,1> action = controller(current_state); //loese MPC Problem


    *steering_front = action(0,0);
    *steering_rear = action(0,1);

    return;

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





