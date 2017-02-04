#include "trajectory_point_follower.h"
#include <cmath>
extern "C"{
#include "andromeda.h"
}


bool TrajectoryPointController::initialize() {
    trajectory = readChannel<street_environment::Trajectory>("TRAJECTORY");
    debugging_trajectoryPoint = writeChannel<street_environment::TrajectoryPoint>("TRAJECTORY_POINT");
    trajectoryDebug = writeChannel<street_environment::Trajectory>("TRAJECTORY_DEBUG");

    car = writeChannel<street_environment::CarCommand>("CAR");

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
    auto phxService = getService<phoenix_CC2016_service::Phoenix_CC2016Service>("PHOENIX_SERVICE");
    if(phxService->driveMode() == phoenix_CC2016_service::CCDriveMode::IDLE){
        street_environment::CarCommand::State *tmp = car->getState("IDLE");
        street_environment::CarCommand::State state;
        if(tmp){
            state = *tmp;
        }
        state.state = street_environment::CarCommand::StateType::IDLE;
        state.priority = 100;
        state.name = "IDLE";
        state.steering_front = 0;
        state.steering_rear = 0;
        state.targetSpeed = 0;
        car->putState(state);
        return true;
    }else{
        car->removeState("IDLE");
    }

    //set the default state
    street_environment::CarCommand::State *tmp = car->getState("DEFAULT");
    street_environment::CarCommand::State state;
    if(tmp){
        state = *tmp;
    }
    state.priority = 10;
    state.name = "DEFAULT";

    //get type
    std::string type = config().get<std::string>("type","tobiMPC");
    if(type == "tobiMPC"){
        float distanceToTrajectoryPoint = m_trajectoryPointDistanceLookup.linearSearch(car->velocity());
        //const float distanceSearched = config().get<float>("distanceRegelpunkt", 0.50);


        if(phxService->driveMode() == phoenix_CC2016_service::CCDriveMode::FOH){
            distanceToTrajectoryPoint = config().get<float>("regelpunktMin", 0.6) + car->velocity()*config().get<float>("regelpunktSlope", 0.1);
            //enableIndicators = false;
        }

        //get the trajectory point
        street_environment::TrajectoryPoint trajectoryPoint = getTrajectoryPoint(distanceToTrajectoryPoint);
        //double v = street_environment::Car::velocity();
        double v = car->velocity();
        if(fabs(v) < 0.1){
            logger.debug("cycle")<<"car is slow: "<<car->velocity();
            v=0.1;//Some controller has some issue divides by v without error-checking
        }

        double phi_soll = atan2(trajectoryPoint.directory.y, trajectoryPoint.directory.x);
        double y_soll = trajectoryPoint.position.y;

        logger.debug("phi y v")<<phi_soll<< " "<< y_soll<<" "<<v;

        double steering_front, steering_rear;

        //von config einlesen, um live einzustellen
       mpcParameters.weight_y = config().get<double>("weight_y",20);
       mpcParameters.weight_phi = config().get<double>("weight_phi",7);
       mpcParameters.weight_steeringFront = config().get<double>("weight_steering_front",0.0005);
       mpcParameters.weight_steeringRear = config().get<double>("weight_steering_rear",10);
       mpcParameters.stepSize = 0.1; //Zeitschrittgroesse fuer MPC
       mpcControllerTobi(v, y_soll, phi_soll, &steering_front, &steering_rear);


        logger.debug("trajectory_point_controller") << "lw vorne: " << steering_front*180/M_PI << "  lw hinten: " << steering_rear*180/M_1_PI;
        if(std::isnan(steering_front) || std::isnan(steering_rear || std::isnan(trajectoryPoint.velocity)) ){
            logger.error("trajectory_point_controller: ")<<"invalid vals: " <<steering_front <<" " <<steering_rear ;
        }

        /*
        //TEST Adaptiver Gierboost
        double yawRate_ist = car->turnRate();
        double yawRate_soll = car->velocity()/0.21*sin(steering_front - steering_rear)/cos(steering_front);
        double yawRate_error = yawRate_soll - yawRate_ist;
        double steeringCorrection = yawRate_error * config().get<float>("yawRateBoost", 0.0);
        steering_front += steeringCorrection;
        */

        state.steering_front = steering_front; // * 180. / M_PI;
        state.steering_rear = steering_rear; // * 180. / M_PI;
        state.targetSpeed = trajectoryPoint.velocity;
        state.targetDistance = trajectoryPoint.position.length(); //TODO absolutwert

        //set trajectoryPoint for debugging;
        *debugging_trajectoryPoint = trajectoryPoint;
        if(trajectoryPoint.velocity == 0){
            state.state = street_environment::CarCommand::StateType::IDLE;
        }else{
            state.state = street_environment::CarCommand::StateType::DRIVING;
        }

    }else if(type == "mikMPC"){
        int delay =  config().get<int>("stagePrediction",0);
        if(delay < 0 || delay >= HORIZON_LEN){
            logger.error("invalid stagePrediction")<<delay;
            return false;
        }
        //get trajectory with distance between points
        logger.time("mikMPC");
        double link_length = config().get<double>("link_length",0.1);
        street_environment::Trajectory tr = trajectory->getWithDistanceBetweenPoints(link_length);
        if(tr.size() < CHAIN_NUM_NODES){
            logger.error("INVALID path given")<< tr.size();
            trajectoryDebug->clear();
            return false;
        }
        tr.erase(tr.begin()+CHAIN_NUM_NODES,tr.end());
        *trajectoryDebug = tr;
        //Inputs
        double currentCarState[NUM_STATES];
        double nodes_x[CHAIN_NUM_NODES];
        double nodes_y[CHAIN_NUM_NODES];
        double nodes_vMin[CHAIN_NUM_NODES-1];
        double nodes_vMax[CHAIN_NUM_NODES-1];
        double max_lateral_acc = config().get<double>("max_lateral_acc",1);
        double max_num_iter = 100;
        double alpha = 0.5;
        double beta_1 = 0.7;
        double beta_2 = 1;
        double q_diag[NUM_STATES];//	stage state cost matrix Q diagonal, len = NUM_STATES
        double r_diag[NUM_INPUTS];//	stage input cost matrix R diagonal, len = NUM_INPUTS
        double p_diag[NUM_STATES];
        const double u_1_ub = config().get<double>("front_angle_rate_Bound",1);
        const double u_1_lb = -u_1_ub;
        const double u_2_ub = config().get<double>("rear_angle_rate_Bound",1);;
        const double u_2_lb = -u_2_ub;

        //car state
        currentCarState[0] = 0;
        currentCarState[1] = 0;
        currentCarState[2] = car->steeringFront();
        currentCarState[3] = car->steeringRear();

        q_diag[0] = config().get<double>("penalty_y",10);
        q_diag[1] = config().get<double>("penalty_phi",10);
        q_diag[2] = config().get<double>("penalty_frontAngle",1);
        q_diag[3] = config().get<double>("penalty_rearAngle",1);
        //path
        for(int i = 0; i < NUM_STATES; i++){
            p_diag[i] = q_diag[i];
        }
        r_diag[0] = config().get<double>("penalty_frontAngle_rate",100);
        r_diag[1] = config().get<double>("penalty_rearAngle_rate",100);


        //set input date
        for(int i = 0; i < CHAIN_NUM_NODES; i++){
            const street_environment::TrajectoryPoint &t = tr[i];
            nodes_x[i] = t.position.x;
            nodes_y[i] = t.position.y;
        }
        for(int i = 0; i < CHAIN_NUM_NODES-1; i++){
            /*
            const street_environment::TrajectoryPoint &t = (*trajectory)[i];
            const street_environment::TrajectoryPoint &t2 = (*trajectory)[i+1];
            nodes_vMax[i] = std::max<double>(t.velocity,t2.velocity);
            nodes_vMin[i] = std::min<double>(t.velocity,t2.velocity);
            */
            //for first tests:
            nodes_vMax[i] = config().get<double>("node_MaxSpeed",0.5);
            nodes_vMin[i] = config().get<double>("node_MinSpeed",0.5);
        }


        //outputs
        double v_star[HORIZON_LEN];
        double u_1_star[HORIZON_LEN];
        double u_2_star[HORIZON_LEN];

        call_andromeda(currentCarState,q_diag,r_diag,p_diag,nodes_x,nodes_y,link_length,nodes_vMin,nodes_vMax,
                       max_lateral_acc,max_num_iter,alpha,beta_1,beta_2,u_1_lb,u_1_ub,u_2_lb,u_2_ub,v_star,u_1_star,u_2_star);

        //Set values
        state.steering_front = car->steeringFront() + u_1_star[delay];
        state.steering_rear = car->steeringRear()+u_2_star[delay];
        state.targetSpeed = v_star[delay];
        state.targetDistance = 1; //TODO dont think that we even need it
        logger.timeEnd("mikMPC");
    }else{
        //use simple pid controller
        float distanceToTrajectoryPoint = m_trajectoryPointDistanceLookup.linearSearch(car->velocity());
        //get the trajectory point
        street_environment::TrajectoryPoint trajectoryPoint = getTrajectoryPoint(distanceToTrajectoryPoint);
        double angleFront = pidControllerFront.pid(trajectoryPoint.position.y);
        double angleRear = pidControllerRear.pid(trajectoryPoint.directory.angle());
        state.steering_front = angleFront;
        state.steering_rear = angleRear;
        state.targetSpeed = trajectoryPoint.velocity;
        state.targetDistance = trajectoryPoint.position.length();
    }

    //set the indicator, sending only once failed CC2016. //TODO better handling
    bool enableIndicators = true;
    state.indicatorLeft  = false;
    state.indicatorRight = false;

    //check if the car is on the left/right
    bool isRight = trajectory->at(0).isRight();
    //check if we change side in the future
    for(const street_environment::TrajectoryPoint &tp:*trajectory){
        if(tp.isRight() != isRight){
            //we will change lane
            state.indicatorLeft  = isRight;
            state.indicatorRight = !isRight;
            logger.debug("turnindicator")<<"left "<< state.indicatorLeft<< ", right "<<state.indicatorLeft;
            break;
        }
    }
    /*
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
    */

    //insert the state
    car->putState(state);
    logger.debug("car state, v, sf, sr")<<state.targetSpeed<<" "<<state.steering_front<<" "<<state.steering_rear;
    return true;
}

void TrajectoryPointController::configsChanged(){
    m_mpcLookupVelocity.vx = config().getArray<float>("mpcLookupVelocityX");
    m_mpcLookupVelocity.vy = config().getArray<float>("mpcLookupVelocityY");
    m_trajectoryPointDistanceLookup.vx = config().getArray<float>("trajectoryPointDistanceLookupX");
    m_trajectoryPointDistanceLookup.vy = config().getArray<float>("trajectoryPointDistanceLookupY");
    slowDownCar.set(config().get<float>("PID_Kp",1),config().get<float>("PID_Ki",0),config().get<float>("PID_Kd",0),config().get<float>("dt",0.01));
    pidControllerFront.set(config().get<float>("PID_front_Kp",1),config().get<float>("PID_front_Ki",1),config().get<float>("PID_front_Kd",0),config().get<float>("dt",0.01));
    pidControllerRear.set(config().get<float>("PID_rear_Kp",1),config().get<float>("PID_rear_Ki",1),config().get<float>("PID_rear_Kd",0),config().get<float>("dt",0.01));
}

void TrajectoryPointController::mpcControllerTobi(double v, double delta_y, double delta_phi, double *steering_front, double *steering_rear) {

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
    //TODO get angle per time

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
    /*
    //could be more stable if the trajectory changes it's direction because of an obstacle etc.
    for(const street_environment::TrajectoryPoint &v:*trajectory){
        if(v.position.length() > distanceToPoint){
            lms::math::vertex2f top = v.position.normalize();
            trajectoryPoint.position = top * distanceToPoint;
            trajectoryPoint.directory = top;
            found = true;
            break;
        }
    }
    */
    //old code
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
                if(std::isnan(velocity) || velocity >= maxVelocityCrossing){
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


