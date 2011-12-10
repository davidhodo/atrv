/*!
 * \file atrv/atrv.h
 * \author David Hodo <david.hodo@gmail.com>
 * \version 0.1
 *
 * \section LICENSE
 *
 * The BSD License
 *
 * Copyright (c) 2011 David Hodo
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * \section DESCRIPTION
 *
 * This provides a cross platform interface for the Roboteq MDC2250 Motor
 * Controller.
 *
 * This library depends on CMake-2.4.6 or later: http://www.cmake.org/
 *
 */

#ifndef ATRV_H
#define ATRV_H

// Standard Library Headers
#include <string>

// MDC2250 library
#include <mdc2250/mdc2250.h>
#include <mdc2250/mdc2250_types.h>

namespace atrv {

/*!
 * Stucture to hold measurements from ATRV motor control boards
 */
struct atrvData {
        // encoder counts
        long wheelCountFL;
        long wheelCountFR;
        long wheelCountRL;
        long wheelCountRR;

        long wheelSpeedFL;
        long wheelSpeedFR;
        long wheelSpeedRL;
        long wheelSpeedRR;
};

/*!
 * Represents an iRobot ATRV powered using Roboteq
 * MDC2250 motor controllers and provides and interface to it.
 */
class ATRV {
public:
    /*!
    * Constructs the ATRV object.
    */
    ATRV();
    virtual ~ATRV();

    /*!
    * Connects to the MDC2250 motor controller given a serial port.
    *
    * \param portFront Defines the serial port for the front motor controller
    * \param portRear Defines the serial port for the rear motor controller
    * Examples: Linux - "/dev/ttyS0" Windows - "COM1"
    *
    * \throws ConnectionFailedException connection attempt failed.
    * \throws UnknownErrorCodeException unknown error code returned.
    */
    bool connect(std::string portFront, std::string portRear);

    /*!
    * Disconnects from the ATRV motor controller
    */
    void disconnect();

    void startReading(int ms, std::string queryString);

    /*!
    * Sets the callback functions for data from the motor controllers
    *
    * \param callFront Pointer to a callback function to process data
    * from the front motor controller
    * \param callRear Pointer to a callback function to process data
    * from the rear motor controller
    *
    * \throws ConnectionFailedException connection attempt failed.
    * \throws UnknownErrorCodeException unknown error code returned.
    */
    void setDataCallback(mdc2250::RuntimeQueryCallback callFront,
                         mdc2250::RuntimeQueryCallback callRear);


    /*!
       * Sends commands to both motor controllers to generate the given
       * linear velocity and yaw rate
       *
       * \param velocity Linear velocity in m/s
       * \param yawRate yaw rad in rad/s
       */
    void setDesiredVehicleMotion(double velocity, double yawRate);

    /*!
       * Sends commands to both motor controllers to generate the given
       * linear velocity and yaw rate
       *
       * \param leftWheel left wheel command
       * \param rightWheel right wheel command
       */
    void setDesiredWheelMotion(long leftWheel, long rightWheel);


    /*!
       * Sets the period between sending commands to the motor controllers
       *
       * \param period time between commands [milliseconds]
       */
    void setControlPeriod(long period){ctrlPeriod=period;}

    long getControlPeriod(){return ctrlPeriod;}

    void setEncoderPPR(long ppr);
    // TODO: this should read the value from the board
    long getEncoderPPR(){return encoderPPR;}

    void setMaxRPM(long rpm);
    // TODO: this should read the value from the board
    long getMaxRPM(){return maxRPM;}


    /*!
       * Sets the vehicle parameters used to convert between
       * linear and angular velocity and wheel speeds
       *
       * \param RL left wheel radius [m]
       * \param Rr right wheel radius [m]
       * \param TW track width [m]
       */
    void setVehicleParameters(double Rl, double Rr, double TW);

    double getLeftRadius(){return radiusL;}
    double getRightRadius(){return radiusR;}
    double getTrackWidth(){return trackWidth;}

    /*!
       * Sends commands the current desired commands to the
       * motor controllers at the given rate
       *
       * \param period time between sending commands in sec
       */
    bool startControlThread();
    //! Stops the control thread which sends commands to the ATRV
    bool stopControlThread();

    bool ESTOP();
    bool clearESTOP();

private:

    // motor controller objects
    mdc2250::MDC2250 mdcFront; //!< interface to the front controller
    mdc2250::MDC2250 mdcRear; //!< interface to the rear controller

    double desiredVel; //!< desired linear velocity [m/s]
    double desiredYawRate; //!< desired yaw rate [rad/s]

    // motor commands are percent of max rpm in closed loop mode
    // and represent a power level in open loop mode
    int desiredLeftCmd;  //!< left motor command (+-1000)
    int desiredRightCmd; //!< right motor command (+-1000)

    ////////////////////////////////////////////////////////////////
    // Vehicle Measurements
    ////////////////////////////////////////////////////////////////
    double radiusL; //!< left wheel radius [m]
    double radiusR; //!< right wheel radius [m]
    double trackWidth; //!< vehicle track width [m]
    long encoderPPR; //!< encoder pulse per revolution setting
    int maxRPM; //!< maximum RPM set on controllers

    //! Method used to continuously send motor commands to ATRV
    void controlThread();

    boost::shared_ptr<boost::thread> mCtrlThread; //!< Boost thread for sending commands to ATRV
    long ctrlPeriod; //!< period [msec] between sending commands to the ATRV
    bool ctrlThreadRunning; //!< used to stop control thread

};



}
#endif
