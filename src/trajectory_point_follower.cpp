#include "trajectory_point_follower.h"
extern "C"{
#include "lenkwinkel.h"
}
#include <cmath>



bool TrajectoryPointController::initialize() {
    trajectory = readChannel<street_environment::Trajectory>("TRAJECTORY");
    debugging_trajectoryPoint = writeChannel<street_environment::TrajectoryPoint>("TRAJECTORY_POINT");

    car = writeChannel<sensor_utils::Car>("CAR");

    //Stellgroessenbeschraenkung
    double alpha_max = 22*M_PI/180;
    lower = -alpha_max, -alpha_max;
    upper =  alpha_max,  alpha_max;

    //myfile.open("mpcData.csv");
    //v_global = 0.0;
    configsChanged();

    isTurn = false;
    turnStart = lms::Time::ZERO;

    return true;
}

bool TrajectoryPointController::deinitialize() {
    return true;
}

bool TrajectoryPointController::cycle() {
    float distanceToTrajectoryPoint = m_trajectoryPointDistanceLookup.linearSearch(car->velocity());
    bool enableIndicators = true;
    //const float distanceSearched = config().get<float>("distanceRegelpunkt", 0.50);


    auto phxService = getService<phoenix_CC2016_service::Phoenix_CC2016Service>("PHOENIX_SERVICE");
    if(phxService->driveMode() == phoenix_CC2016_service::CCDriveMode::FOH){
        distanceToTrajectoryPoint = config().get<float>("regelpunktMin", 0.6) + car->velocity()*config().get<float>("regelpunktSlope", 0.1);
        enableIndicators = false;
    }
    if(phxService->driveMode() == phoenix_CC2016_service::CCDriveMode::IDLE){

        sensor_utils::Car::State *tmp = car->getState("IDLE");
        sensor_utils::Car::State state;
        if(tmp){
            state = *tmp;
        }
        state.state = sensor_utils::Car::StateType::IDLE;
        state.priority = 100;
        state.name = "IDLE";
        state.steering_front = 0;
        state.steering_rear = 0;
        state.targetSpeed = 0;
    }else{
        car->removeState("IDLE");
    }

    street_environment::TrajectoryPoint trajectoryPoint = getTrajectoryPoint(distanceToTrajectoryPoint);
    //double v = sensor_utils::Car::velocity();
    double v = car->velocity();
    if(fabs(v) < 0.1){
        logger.debug("cycle")<<"ar is slow: "<<car->velocity();
        v=0.1;//Some controller has some issue divides by v without error-checking
    }

    double phi_soll = atan2(trajectoryPoint.directory.y, trajectoryPoint.directory.x);
    double y_soll = trajectoryPoint.position.y;

    //logger.error("phi_soll")<<phi_soll<< " "<< y_soll;

    double steering_front, steering_rear;

    if(config().get<bool>("useMPCcontroller",true)){
            //von config einlesen, um live einzustellen
           mpcParameters.weight_y = config().get<double>("weight_y",20);
           mpcParameters.weight_phi = config().get<double>("weight_phi",7);
           mpcParameters.weight_steeringFront = config().get<double>("weight_steering_front",0.0005);
           mpcParameters.weight_steeringRear = config().get<double>("weight_steering_rear",10);
           mpcParameters.stepSize = 0.1; //Zeitschrittgroesse fuer MPC
           mpcController(v, y_soll, phi_soll, &steering_front, &steering_rear);
    }else{
        lenkwinkel(trajectoryPoint.position.length(),y_soll,phi_soll,1,&steering_rear,&steering_front);
    }
    logger.debug("trajectory_point_controller") << "lw vorne: " << steering_front*180/M_PI << "  lw hinten: " << steering_rear*180/M_1_PI;
    if(isnan(steering_front) || isnan(steering_rear || isnan(trajectoryPoint.velocity)) ){
        logger.error("trajectory_point_controller: ")<<"invalid vals: " <<steering_front <<" " <<steering_rear ;
    }

    //TEST Adaptiver Gierboost
    double yawRate_ist = car->turnRate();
    double yawRate_soll = car->velocity()/0.21*sin(steering_front - steering_rear)/cos(steering_front);
    double yawRate_error = yawRate_soll - yawRate_ist;
    double steeringCorrection = yawRate_error * config().get<float>("yawRateBoost", 0.0);
    steering_front += steeringCorrection;


    //debug-----
    //myfile << steering_front << "," << steering_rear << "," << v << std::endl;
    //----------

    //set the default state
    sensor_utils::Car::State *tmp = car->getState("DEFAULT");
    sensor_utils::Car::State state;
    if(tmp){
        state = *tmp;
    }
    state.priority = 10;
    state.name = "DEFAULT";

    state.steering_front = steering_front; // * 180. / M_PI;
    state.steering_rear = steering_rear; // * 180. / M_PI;
    state.targetSpeed = trajectoryPoint.velocity;
    if(trajectoryPoint.velocity == 0){
        state.state = sensor_utils::Car::StateType::IDLE;
    }else{
        state.state = sensor_utils::Car::StateType::DRIVING;
    }

    logger.debug("positionController")<<"dv: "<<state.steering_front<< " dh"<<state.steering_rear<<" vel: "<<state.targetSpeed;
    //set the indicator
    //get the closest change
    //set the indicator
    //float indicatorMaxDistance = config().get<float>("indicatorMaxDistance",0.5);
    state.indicatorLeft  = false;
    state.indicatorRight = false;

    if(enableIndicators && trajectory->size() > 1) {
        bool isRight = trajectory->at(1).isRight();
        size_t i = 0;
        for(const street_environment::TrajectoryPoint &tp:*trajectory){
            if(i++ == 0) {
                continue;
            }
            if(isRight) {
                if(!tp.isRight()){
                    // Lane change right -> left
                    state.indicatorLeft  = true;
                    state.indicatorRight = false;
                    isTurn = true;
                    turnStart = lms::Time::now();
                    break;
                }
            }
        }

        if(isTurn) {
            if(turnStart.since() > lms::Time::fromMillis(700)) {
                isTurn = false;
                state.indicatorLeft  = false;
                state.indicatorRight = false;
            }else if(turnStart.since() > lms::Time::fromMillis(400)) {
                state.indicatorLeft  = false;
                state.indicatorRight = true;
            }
        }
    }

    //insert the state
    car->putState(state);

    //set trajectoryPoint for debugging;
    *debugging_trajectoryPoint = trajectoryPoint;
    return true;
}

void TrajectoryPointController::configsChanged(){
    m_mpcLookupVelocity.vx = config().getArray<float>("mpcLookupVelocityX");
    m_mpcLookupVelocity.vy = config().getArray<float>("mpcLookupVelocityY");
    m_trajectoryPointDistanceLookup.vx = config().getArray<float>("trajectoryPointDistanceLookupX");
    m_trajectoryPointDistanceLookup.vy = config().getArray<float>("trajectoryPointDistanceLookupY");
    slowDownCar.set(config().get<float>("PID_Kp",1),config().get<float>("PID_Ki",0),config().get<float>("PID_Kd",0),config().get<float>("dt",0.01));

}

void TrajectoryPointController::mpcController(double v, double delta_y, double delta_phi, double *steering_front, double *steering_rear) {

    const int STATES = 2; //number of states (y and phi)
    const int CONTROLS = 2; //number of control inputs (steering_front and steering_rear)
    double T = mpcParameters.stepSize;

    // Modell festlegen
    // x_{i+1} == A*x_i + B*u_i + C

    //HACK:
    //anstatt v konstant zu lassen folgende Idee:
    //Wenn v größer wird, kann im betrachteten Prädiktionshorizont das Regelziel schneller erreicht werden.
    //Das führt dann (meiner Meinung nach) tendenziell zu kleineren Lenkwinkeln. Wir wollen aber eigentlich
    //genau das Gegenteil, nämlich größere Lenkwinkel bei höherer Geschwindigkeit, um Querschlupf entgegenzuwirken.
    //Lösungsansatz:
    //Vorgabe einer Grundgeschwindigkeit v0 (z.B. v0=1), auf die der Regler bei langsamer Fahrt eingestellt wird.
    //Dann abhängig von der wirklichen aktuellen Geschwindigkeit Addition eines "Geschwindigkeitsfaktors"
    // ==> v_regler = 1 + c*v_real oder alternativ v_regler = exp(-c*v_real)

    //if (config().get("velocityFactor", 0.0)) v = std::exp(-v*config().get("velocityFactor", 0.0));
    //v = std::max(1.0, v);

    v = m_mpcLookupVelocity.linearSearch(v);


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


    dlib::mpc<STATES,CONTROLS,MPC_HORIZON> controller(A,B,C,Q,R,lower,upper); //30*T ist der Zeithorizont fuer die praediktion, d.h. 30 Zeitschritte wird in die Zukunft simuliert

    dlib::matrix<double,STATES,1> target;
    target = delta_y, delta_phi;

    controller.set_target(target);

    //Stellschrauben fuer performance
    //controller.set_epsilon(0.05);
    //controller.set_max_iterations(300);

    dlib::matrix<double,STATES,1> current_state;
    current_state = 0, 0;

    dlib::matrix<double,CONTROLS,1> action = controller(current_state); //loese MPC Problem


    *steering_front = action(0,0);
    *steering_rear = action(1,0);

    return;

}


street_environment::TrajectoryPoint TrajectoryPointController::getTrajectoryPoint(const float distanceToPoint){
    //if we find nothing, we just want to idle forward
    street_environment::TrajectoryPoint trajectoryPoint;
    //x-Pos
    trajectoryPoint.position.x = distanceToPoint;
    //y-Pos
    trajectoryPoint.position.y = 0;
    //x-Dir
    trajectoryPoint.directory.x = 1;
    //y-Dir
    trajectoryPoint.directory.y = 0;
    trajectoryPoint.velocity = 0;
    if(trajectory->size()  == 0){
        logger.warn("cycle") <<"Can't follow anything";
        return trajectoryPoint;
    }
    bool found = false;
    //Nur den Abstand in x-richtung zu nehmen ist nicht schlau, denn wenn das Auto eskaliert eskaliert der Regler noch viel mehr!
    float currentDistance = 0;
    for(int i = 1; i < (int)trajectory->size();i++){
        street_environment::TrajectoryPoint bot = trajectory->at(i-1);
        street_environment::TrajectoryPoint top = trajectory->at(i);
        currentDistance += bot.position.distance(top.position);
        if(currentDistance > distanceToPoint){
            //We start at the bottom-point
            //inerpolate between bot and top! #IMPORTANT (velocity!,viewdir)
            float delta = currentDistance-distanceToPoint;
            lms::math::vertex2f along = (bot.position-top.position).normalize()*delta;
            trajectoryPoint =  top;
            trajectoryPoint.position = trajectoryPoint.position+along;
            found = true;
            break;
        }
    }
    if(!found){
        logger.warn("No trajectoryPoint found, returning the last point of the trajectory")<<"trajPointCount"<<trajectory->size()<< " distanceSearched: "<< currentDistance << " distanceToTrajectoryPoint: "<< distanceToPoint;
        trajectoryPoint = trajectory->at(trajectory->size()-1);
    }

    //HACK stop at crossing
    float minVelocity = config().get<float>("maxVelocityCrossing", 1.0);
    for(const street_environment::TrajectoryPoint &v:*trajectory){
        if(v.velocity == 0){
            float distanceToStop = lms::math::sgn(v.position.x)*v.position.length() - config().get<float>("stoppingDistance",0.35);
            if(distanceToStop < config().get<float>("distanceToStop",1)){
                logger.debug("distanceToStop")<< distanceToStop;
                float maxVelocityCrossing = config().get<float>("maxVelocityCrossing", 1.0);
                float velocity = slowDownCar.pid(distanceToStop);
                if(isnan(velocity) || velocity >= maxVelocityCrossing){
                    velocity = maxVelocityCrossing;
                }
                if(distanceToStop<= config().get<float>("crossingSaftyZone",0.05) || velocity < 0){
                    velocity = 0.0;
                }
                if(velocity < minVelocity){
                    minVelocity = velocity;
                }

                trajectoryPoint.velocity = minVelocity;
                //0 is our min velocity
                if(minVelocity == 0){
                    break;
                }
            }else{
                slowDownCar.reset();
            }
        }
    }
    //we just return the last Point
    return trajectoryPoint;
}


