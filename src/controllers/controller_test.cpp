#include "impl/PID.hpp"
#include "impl/bang_bang.hpp"
#include "impl/double_setpoint.hpp"
#include <gtest/gtest.h>
#include <memory>


class ControllerTest : public ::testing::TestWithParam<std::shared_ptr<Controller>> {
protected:
    void SetUp() override {}

    void TearDown() override {}
};

std::vector<std::shared_ptr<Controller>> getMethodsToTest()
{
    std::vector<std::shared_ptr<Controller>> controllers;
    controllers.push_back(std::shared_ptr<Controller>(new controllers::PID(0.1,0.001,0.001)));
    controllers.push_back(std::shared_ptr<Controller>(new controllers::BangBang(0.1,-0.1,0.1)));
    controllers.push_back(std::shared_ptr<Controller>(new controllers::DoubleSetpoint(0.1,0.0,-0.1,0.1)));
    return controllers;
}

//Basic controller output should be same sign like error
TEST_P(ControllerTest, TestConstFunction)
{
    constexpr double val = 5.0;

    auto controller = GetParam();

    controller->clear();
    ASSERT_GE(controller->calc(val),0.0);
    controller->clear();
    ASSERT_LE(controller->calc(-val),0.0);
}

//Control simple dynamic object
TEST_P(ControllerTest, SimpleObjectControl)
{
    constexpr double x0 = 0.0;
    constexpr double x_set = 10.0;
    constexpr double dt = 0.1;
    constexpr double control_time = 10.0;
    constexpr int loops = 3;
    constexpr double tol = 0.2;

    auto controller = GetParam();
    controller->clear();
    controller->set_dt(dt);

    double x = x0;

    for(int loop = 0; loop < loops; loop++)
    {
        for (int i = 0; i < static_cast<int>(control_time/dt); i++)
        {
            auto output = controller->calc(x_set - x);
            x += output;
        }
        EXPECT_NEAR(x,x_set,tol);
    }
}

INSTANTIATE_TEST_SUITE_P(TestDerivedClasses, ControllerTest, testing::ValuesIn(getMethodsToTest()));

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}