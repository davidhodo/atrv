#include "gtest/gtest.h"

// OMG this is so nasty...
#define private public
#define protected public

#include "atrv/atrv.h"
using namespace atrv;

namespace {

class CommandTests : public ::testing::Test {
protected:
    virtual void SetUp() {
        myAtrv.setVehicleParameters(0.203,0.203,0.76);
        myAtrv.setMaxRPM(3000);
        myAtrv.setEncoderPPR(500);
    }
    
    ATRV myAtrv;
};

TEST_F(CommandTests, DriveStraight) {
    myAtrv.setDesiredVehicleMotion(1.0,0.0);
    EXPECT_EQ(172, myAtrv.desiredLeftCmd);
    EXPECT_EQ(172, myAtrv.desiredRightCmd);
}

TEST_F(CommandTests, DriveStraight2) {
    myAtrv.setDesiredVehicleMotion(0.5,0.0);
    EXPECT_EQ(86, myAtrv.desiredLeftCmd);
    EXPECT_EQ(86, myAtrv.desiredRightCmd);
}

TEST_F(CommandTests, TurnInPlace) {
    myAtrv.setDesiredVehicleMotion(0.0,1.0);
    EXPECT_EQ(65, myAtrv.desiredLeftCmd);
    EXPECT_EQ(-65, myAtrv.desiredRightCmd);
}

}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
