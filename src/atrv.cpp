#include "atrv/atrv.h"
using namespace atrv;

ATRV::ATRV() {
    // initialize variables
    ctrlThreadRunning=false;
    ctrlPeriod=100; // default to 10Hz control
    desiredVel=0;
    desiredYawRate=0;
    desiredLeftCmd=0;
    desiredRightCmd=0;
    radiusL=0.203;
    radiusR=0.203;
    trackWidth=0.76;
    maxRPM=3000;
    encoderPPR=500;
}

ATRV::~ATRV() {
    disconnect();
}

bool ATRV::connect(std::string portFront, std::string portRear) {
    bool result;
    result=mdcFront.connect(portFront);
    if (!result) {
        std::cout << "ATRV: Failed to connect to front motor controller." << std::endl;
        return false;
    }

    result=mdcRear.connect(portRear);
    if (!result) {
        std::cout << "ATRV: Failed to connect to rear motor controller." << std::endl;
        mdcFront.disconnect();
        return false;
    }

    setEncoderPPR(encoderPPR);
    setMaxRPM(maxRPM);

    return true;
}

void ATRV::disconnect() {
    // disconnect both motor controllers and close serial ports
    mdcFront.disconnect();
    mdcRear.disconnect();
}

void ATRV::startReading(int ms, std::string queryString) {
    // example: requests battery amps, fault flags, speed, and encoder counts at 5Hz
    //myMDC.sendCommand("\r# C_?BA_?FF_?S_?C_# 200\r");
    std::stringstream cmd;
    // form command string
    // format:
    // "\r" - start with carriage return to clear any previous commands
    // "# C" - clear the query history
    // "?Q1_?Q2_?Q3_..." - list of query items: Q1, Q2, ...
    // "# 10" - request the queries repeat at 10 millisecond intervals
    // "\r" - complete the command
    cmd << "\r# C_" << queryString << "# " << ms << "\r";
    mdcFront.sendCommand(cmd.str());
    mdcRear.sendCommand(cmd.str());
    // start the read thread to listen for automatically sent queries
    mdcFront.startContinuousReading();
    mdcRear.startContinuousReading();
}


void ATRV::setDataCallback(mdc2250::RuntimeQueryCallback callFront, mdc2250::RuntimeQueryCallback callRear) {
    mdcFront.setRuntimeQueryCallback(callFront);
    mdcRear.setRuntimeQueryCallback(callRear);
}

bool ATRV::startControlThread() {
        ctrlThreadRunning=true;
        mCtrlThread = boost::shared_ptr<boost::thread > (new boost::thread(boost::bind(&ATRV::controlThread, this)));
        std::cout << "ATRV: Control thread started. Sending commands to MDC2250." << std::endl;

        // TODO: check to see if thread is running before returning
        return true;
}

bool ATRV::stopControlThread() {
    ctrlThreadRunning=false;
    // TODO: check to see if thread has stopped running before returning
    return true;
}

void ATRV::controlThread() {
        while (ctrlThreadRunning) {
                // send commands to the front and rear motors
                // channel 1 is left motor, channel 2 is right motor
                mdcFront.multiMotorCmd(desiredLeftCmd,desiredRightCmd);
                mdcRear.multiMotorCmd(desiredLeftCmd,desiredRightCmd);
                // wait X
                boost::this_thread::sleep(boost::posix_time::milliseconds(ctrlPeriod));
                // TODO: modify this later to take into account amount of time between waits
        }

        std::cout << "ATRV: Control thread stopped." << std::endl;
}


void ATRV::setVehicleParameters(double Rl, double Rr, double TW) {

    radiusL=Rl;
    radiusR=Rr;
    trackWidth=TW;
}

void ATRV::setDesiredVehicleMotion(double velocity, double yawRate){
        // calculate the desired wheel commands from the given velocity
        // and yaw rate using the vehicle parameters

        // calculate desired wheel speeds in [rpm]
        double leftWheelSpeed=60*(2*velocity+yawRate*trackWidth)/(4*3.1416*radiusL);
        double rightWheelSpeed=	60*(2*velocity-yawRate*trackWidth)/(4*3.1416*radiusR);

        // calculate rpm as percentage of max rpm
        desiredLeftCmd=(leftWheelSpeed/(double)maxRPM)*1000.0;
        desiredRightCmd=(rightWheelSpeed/(double)maxRPM)*1000.0;

}

void ATRV::setDesiredWheelMotion(long leftWheel, long rightWheel) {
        desiredLeftCmd=leftWheel;
        desiredRightCmd=rightWheel;
}

bool ATRV::ESTOP() {
    mdcFront.ESTOP();
    mdcRear.ESTOP();
    return true;
}

bool ATRV::clearESTOP() {
    mdcFront.ClearESTOP();
    mdcRear.ClearESTOP();
    return true;
}

void ATRV::setEncoderPPR(long ppr) {
    // TODO: later change these sleeps to waitForAck()
    mdcFront.setEncoderPPR(1,ppr);
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    mdcFront.setEncoderPPR(2,ppr);
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    mdcRear.setEncoderPPR(1,ppr);
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    mdcRear.setEncoderPPR(2,ppr);
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
}

void ATRV::setMaxRPM(long rpm) {
    // TODO: later change these sleeps to waitForAck()
    mdcFront.setMaxRPM(1,rpm);
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    mdcFront.setMaxRPM(2,rpm);
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    mdcRear.setMaxRPM(1,rpm);
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    mdcRear.setMaxRPM(2,rpm);
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
}
