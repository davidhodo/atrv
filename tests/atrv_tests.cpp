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

TEST_F(CommandTests, SetVehicleMotion) {
    myAtrv.setDesiredVehicleMotion(1.0,0.0);
    EXPECT_EQ(15, myAtrv.desiredLeftCmd);
    EXPECT_EQ(15, myAtrv.desiredRightCmd);
}

}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
