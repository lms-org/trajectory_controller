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

    return true;
}

bool TrajectoryPointController::deinitialize() {
    return true;
}

bool TrajectoryPointController::cycle() {
    const float distanceToTrajectoryPoint = m_trajectoryPointDistanceLookup.linearSearch(car->velocity());
    //const float distanceSearched = config().get<float>("distanceRegelpunkt", 0.50);

    street_environment::TrajectoryPoint trajectoryPoint = getTrajectoryPoint(distanceToTrajectoryPoint);
    //double v = sensor_utils::Car::velocity();
    double v = car->velocity();
    if(fabs(v) < 0.1){
        logger.debug("cycle")<<"velocity is 0";
        v=0.1;//Some controller has some issue divides by v without error-checking
    }


    double phi_soll = atan2(trajectoryPoint.directory.y, trajectoryPoint.directory.x);
    double y_soll = trajectoryPoint.position.y;

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
    state.steering_rear = -state.steering_rear; //TODO HACK

    logger.debug("positionController")<<"dv: "<<state.steering_front<< " dh"<<state.steering_rear<<" vel: "<<state.targetSpeed;
    //set the indicator
    //get the closest change
    float nextChangeDistance = config().get<float>("indicatorMaxDistance",0.5);
    for(const street_environment::Trajectory::RoadChange &change : trajectory->changes){
        float xDistance = trajectory->points()[change.changeRoadIndex].x;
        if(xDistance < config().get<float>("indicatorMinDistance",0)){
            continue;
        }else if(xDistance >= nextChangeDistance){
            continue;
        }
        nextChangeDistance = xDistance;
        if(change.changeToLeft){
            state.indicatorLeft = true;
            state.indicatorRight = false;
        }else{
            state.indicatorRight = true;
            state.indicatorLeft = false;
        }
    }
    //insert the state
    car->putState(state);

    //set trajectoryPoint for debugging;
    *debugging_trajectoryPoint = trajectoryPoint;
    return true;
}
float TrajectoryPointController::targetVelocity(){
    float velocity = 0;
    float maxForcastLength = config().get<float>("maxForcastLength",1);
    float minForcastLength = config().get<float>("minForcastLength",0.3);
    float targetForcastLength = config().get<float>("targetForcastLength",0.6);
    float maxAngle = config().get<float>("maxAngle",0.6);
    float maxSpeed = config().get<float>("maxSpeed",1);
    float minCurveSpeed = config().get<float>("minSpeed",maxSpeed/2);
    float weightOffset = config().get<float>("weightOffset",1);
    float weightSlope = config().get<float>("weightSlope",1);

    if(maxForcastLength > trajectory->length()){
        slowDownCar.set(config().get<float>("PID_Kp",1),config().get<float>("PID_Ki",0),config().get<float>("PID_Kd",0),config().get<float>("dt",0.01));
        //road will come to an end
        //reduce speed, we drive backwards if we went to far!
        velocity = slowDownCar.pid(trajectory->length()*lms::math::sgn<float>(trajectory->points()[trajectory->points().size()-1].x));
    }else{
        //reset the PID controller
        slowDownCar.reset();
        //TODO Momentan ist es wichtig, dass die Trajectorie sehr fein ist!
        //TODO that was stupud lms::math::polyLine2f tempTraj = trajectory->getWithDistanceBetweenPoints(config().get<float>("distanceBetweenTrajectoryPoints",0.05));
        float currentDistance = 0;
        float totalWeight = 0;
        float angle = 0;
        for(int i = 1; i <(int) trajectory->points().size(); i++){
            lms::math::vertex2f bot = trajectory->points()[i-1];
            lms::math::vertex2f top = trajectory->points()[i];
            currentDistance += bot.distance(top);

            if(currentDistance < minForcastLength){
                continue;
            }else if(currentDistance > maxForcastLength){
                break;
            }
            if(bot.length() == 0 && top.length()==0){
                angle = 0;
            }else{
                float newAngle = 0;
                if(bot.length() == 0){
                    newAngle = top.angle();
                }else{
                    newAngle = bot.angleBetween(top);
                }
                newAngle = fabs(newAngle);

                /*
                //wir suchen den max angle
                if(newAngle > angle){
                    angle = newAngle;
                }
                totalWeight = 1;
                */
                float weight = weightOffset;

                //wir nehmen den gewichteten average
                if(currentDistance <= targetForcastLength){
                    weight += weightSlope+(1-(targetForcastLength-currentDistance)/(targetForcastLength-minForcastLength));
                }else{
                    weight += weightSlope+(1-(currentDistance-targetForcastLength)/(maxForcastLength-targetForcastLength));
                }
                totalWeight += weight;
                angle += newAngle*weight;//quadrierter wert
            }
        }
        velocity = (minCurveSpeed-maxSpeed)/maxAngle*(angle/totalWeight)+maxSpeed;

        logger.debug("velocity")<<"angle: "<<angle/totalWeight<<" maxAngle: "<<maxAngle;
        logger.debug("velocity")<<"velocity: "<<velocity;

    }
    if(isnan(velocity)){
        logger.error("targetVelocity")<<"velocity is NAN";
        velocity = 0;
    }
    return velocity;
}

void TrajectoryPointController::configsChanged()
{
    m_mpcLookupVelocity.vx = config().getArray<float>("mpcLookupVelocityX");
    m_mpcLookupVelocity.vy = config().getArray<float>("mpcLookupVelocityY");
    m_trajectoryPointDistanceLookup.vx = config().getArray<float>("trajectoryPointDistanceLookupX");
    m_trajectoryPointDistanceLookup.vy = config().getArray<float>("trajectoryPointDistanceLookupY");
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
    //else v = std::max(1.0, v);

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


street_environment::TrajectoryPoint TrajectoryPointController::getTrajectoryPoint(float distanceToPoint){
    street_environment::TrajectoryPoint trajectoryPoint;
    bool found = false;
    if(trajectory->points().size()  == 0){
        logger.warn("cycle") <<"Can't follow anything";
    }else if(trajectory->points().size() == 1){
        //set endPoint
        found = true;
        trajectoryPoint.directory = lms::math::vertex2f(1,0);
        trajectoryPoint.position = trajectory->points()[0];
        trajectoryPoint.velocity = 0;
        found = true;
    }
    for(int i = 1; i < (int)trajectory->points().size();i++){
        //TODO put 0.2 in config
        lms::math::vertex2f top = trajectory->points()[i];
        //logger.debug("cycle") << "pos: " << toFollow->points()[i].x() << " " << toFollow->points()[i].y();
        if(top.x > distanceToPoint){
            //TODO has to be tested
            //We start at the bottom-point
            lms::math::vertex2f bot = trajectory->points()[i-1];
            float toGoX = distanceToPoint-bot.x;
            if(toGoX <= 0){
                //TODO
                toGoX = 0;
            }

            float angle = (top-bot).angle();
            //we to the bot-coords the length, that is still to go in absolute direction to 0,0
            //TODO point isn't on the trajectory but I think it could be fine
            //x-Pos
            trajectoryPoint.position.x = bot.x + toGoX*cos(angle);
            //y-Pos
            trajectoryPoint.position.y = bot.y + toGoX*sin(angle);
            if(i >= (int)trajectory->viewDirs.points().size()){
                //x-Dir
                trajectoryPoint.directory.x = cos(angle);
                //y-Dir
                trajectoryPoint.directory.y = sin(angle);
                logger.error("getTrajectoryPoint")<<"no viewDir given! "<< trajectory->viewDirs.points().size();
            }else{
                lms::math::vertex2f dir = trajectory->viewDirs.points()[i].normalize();

                //check if the viewDir can be used
                if(dir.angle() > config().get<float>("maxParallelAusweichen",22.0*M_PI/180.0)){//TODO naming
                    //x-Dir
                    trajectoryPoint.directory.x = cos(angle);
                    //y-Dir
                    trajectoryPoint.directory.y = sin(angle);
                    logger.warn("maxParallelAusweichen")<<"using trajectoryViewDir, not the viewDir!";
                }
                trajectoryPoint.directory.x = dir.x;
                trajectoryPoint.directory.y = dir.y;
            }
            trajectoryPoint.velocity = targetVelocity();
            found = true;
            break;
        }
    }
    if(!found){
        //if we find nothing, we just want to idle forward
        //x-Pos
        trajectoryPoint.position.x = distanceToPoint;
        //y-Pos
        trajectoryPoint.position.y = 0;
        //x-Dir
        trajectoryPoint.directory.x = 1;
        //y-Dir
        trajectoryPoint.directory.y = 0;
        trajectoryPoint.velocity = 0;
    }
    return trajectoryPoint;
}


