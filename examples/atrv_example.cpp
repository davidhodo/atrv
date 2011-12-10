#include <string>
#include <iostream>

#include "atrv/atrv.h"
using namespace atrv;
using namespace std;
using namespace mdc2250;

void defaultRuntimeQueryCallback(mdc2250_status status, RuntimeQuery::runtimeQuery queryType) {
    std::cout << "Parsed runtime query: " << queryType << std::endl;
}

int main(int argc, char **argv)
{
    if(argc < 3) {
        std::cerr << "Usage: atrv_example <serial port address> <serial port address>" << std::endl;
        return 0;
    }
    std::string portFront(argv[1]);
    std::string portRear(argv[2]);

    std::cout << "WARNING: This example moves the ATRV. \n Do you wish to continue (Y or N)?" << std::endl;
    char reply;
    std::cin >> reply;

    ATRV myATRV;
    // wheel diameter 16"
    // track width ~2.5'
    myATRV.setVehicleParameters(0.203,0.203,0.76);

    bool result=myATRV.connect(portFront, portRear);

    if (result) {
        cout << "Successfully connected." << endl;
    }
    else {
        cout << "Failed to connect." << endl;
        return -1;
    }

    // set up to read BA, FF, S, CR
    myATRV.startReading(200,"C_?BA_?FF_?S_?C_");

    if (reply=='Y') {

        myATRV.setControlPeriod(50);  // send commands at 20Hz
        myATRV.startControlThread();

        // run motor
        for (int ii=0; ii<20; ii++)
        {
            //std::cout << ii << std::endl;
            myATRV.setDesiredWheelMotion(400,400);
            boost::this_thread::sleep(boost::posix_time::milliseconds(100));
        }
        for (int ii=0; ii<20; ii++)
        {
            //std::cout << ii << std::endl;
            myATRV.setDesiredWheelMotion(100,100);
            boost::this_thread::sleep(boost::posix_time::milliseconds(100));
        }
        for (int ii=0; ii<20; ii++)
        {
            //std::cout << ii << std::endl;
            myATRV.setDesiredWheelMotion(400,400);
            boost::this_thread::sleep(boost::posix_time::milliseconds(100));
        }

        myATRV.setDesiredWheelMotion(0,0);
    } else {
        std::cout << "Running without motion." << std::endl;
    }

    while(1);

    myATRV.disconnect();


    return 0;
}
