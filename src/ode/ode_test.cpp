#include "ode.hpp"
#include <gtest/gtest.h>
#include <numbers>


class ODETest : public ::testing::TestWithParam<ODE::ODEMethod> {
protected:
    void SetUp() override {}

    void TearDown() override {}
};

std::vector<ODE::ODEMethod> getMethodsToTest()
{
    std::vector<ODE::ODEMethod> methods;
    for (int i = ODE::ODEMethod::Euler; i < ODE::ODEMethod::NONE; i++)
    {
        methods.push_back(static_cast<ODE::ODEMethod>(i));
    }
    return methods;
}

//Every ODE should be parsed from string
TEST_F(ODETest, FromStringTest) {
    ODE::ODEMethod method = ODE::fromString("Euler");
    ASSERT_EQ(method, ODE::Euler);

    method = ODE::fromString("Heun");
    ASSERT_EQ(method, ODE::Heun);

    method = ODE::fromString("RK4");
    ASSERT_EQ(method, ODE::RK4);

    method = ODE::fromString("PC2");
    ASSERT_EQ(method, ODE::PC2);

    method = ODE::fromString("PC4");
    ASSERT_EQ(method, ODE::PC4);

    method = ODE::fromString("Invalid");
    ASSERT_EQ(method, ODE::NONE);
}

//Every ODE should be properly create by factory
TEST_F(ODETest, FactoryTest) {
    std::unique_ptr<ODE> eulerODE = ODE::factory(ODE::Euler);
    ASSERT_NE(eulerODE, nullptr); 

    std::unique_ptr<ODE> heunODE = ODE::factory(ODE::Heun);
    ASSERT_NE(heunODE, nullptr);

    std::unique_ptr<ODE> rk4ODE = ODE::factory(ODE::RK4);
    ASSERT_NE(rk4ODE, nullptr);

    std::unique_ptr<ODE> pc2ODE = ODE::factory(ODE::PC2);
    ASSERT_NE(pc2ODE, nullptr);

    std::unique_ptr<ODE> pc4ODE = ODE::factory(ODE::PC4);
    ASSERT_NE(pc4ODE, nullptr);

    std::unique_ptr<ODE> invalidODE = ODE::factory(ODE::NONE);
    ASSERT_EQ(invalidODE, nullptr);
}

//ODE should not modify y if derivative is zero
TEST_P(ODETest, TestConstFunction)
{
    auto method = GetParam();
    auto ode = ODE::factory(method);
    ASSERT_NE(ode, nullptr);

    Eigen::Vector3d y0;
    y0 << 1.0, 2.0, 3.0;
    auto y = ode->step(0.0,y0,[](double t, Eigen::VectorXd y){
            return Eigen::Vector3d::Zero();
        },1.0);

    ASSERT_FLOAT_EQ((y-y0).norm(),0.0);
}

//ODE should be at list first order numerical procedure
TEST_P(ODETest, TestFirstOrder)
{
    constexpr double a = 2.0;
    constexpr int t = 5;
    constexpr double tol = 1e-10;

    auto method = GetParam();
    auto ode = ODE::factory(method);
    ASSERT_NE(ode, nullptr);

    Eigen::Vector<double,1> y0;
    y0 << 0.0;
    Eigen::Vector<double,1> y = y0;
    for(int i = 0; i < t; i++)
    {
        double time = i * 1.0;
        y = ode->step(time,y,[](double t, Eigen::VectorXd y){
            return Eigen::Vector<double,1>::Constant(2.0);
        },1.0);
    }
    EXPECT_NEAR(y.x() - a * t,0.0,tol);
}

//RHS function should be called declared times
TEST_P(ODETest, TestRHSCalls)
{
    auto method = GetParam();
    auto ode = ODE::factory(method);
    ASSERT_NE(ode, nullptr);

    int counter = 0;
    Eigen::Vector<double,1> y0;
    y0 << 0.0;
    //run it 10 times for non self-starting methods
    for(int i = 0; i < 10; i++)
    {
        ode->step(0.0,y0,
            [](double t, Eigen::VectorXd y)
            {
                return y;
            },1.0);
    }
    ode->step(0.0,y0,
        [&counter](double t, Eigen::VectorXd y)
        {
            counter++;
            return y;
        },1.0);

    ASSERT_EQ(ODE::getMicrosteps(method),ode->getMicrosteps());
    ASSERT_EQ(counter, ode->getMicrosteps());
}

INSTANTIATE_TEST_SUITE_P(TestDerivedClasses, ODETest, testing::ValuesIn(getMethodsToTest()));

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
