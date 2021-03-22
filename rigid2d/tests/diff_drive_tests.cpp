#include <catch_ros/catch.hpp>
#include <rigid2d/rigid2d.hpp>
#include <rigid2d/diff_drive.hpp>

/// \brief testing function that updates configuration of the diff drive robot
TEST_CASE("Update robot configuration", "[update configuration]") // Sarah, Ziselman
{
    using namespace rigid2d; 
    double wheelbase = 2;
    double wheelradius = 1;

    // move forwards 
    DiffDrive robot = DiffDrive(wheelbase, wheelradius, 0.0, 0.0, 0.0, 0.0, 0.0);

    double leftWheelNew = PI/2;
    double rightWheelNew = PI/2;
    robot(leftWheelNew, rightWheelNew);

    REQUIRE(robot.getTh() == Approx(0));
    REQUIRE(robot.getX() == Approx(PI/2));
    REQUIRE(robot.getY() == Approx(0));
}

// TEST_CASE("Update robot configuration 2", "[update configuration 2]") // Sarah, Ziselman
// {
//     using namespace rigid2d;
//     double wheelbase = 2;
//     double wheelradius = 1;

//     DiffDrive robot = DiffDrive(wheelbase, wheelradius, 0.0, 0.0, 0.0, 0.0, 0.0);

//     double leftWheelNew = PI;
//     double rightWheelNew = PI/2;
//     robot(leftWheelNew, rightWheelNew);

//     REQUIRE(robot.getTh() == Approx(-PI/4));
//     REQUIRE(robot.getX() == Approx(3/sqrt(2)));
//     REQUIRE(robot.getY() == Approx((3 + 3*sqrt(2))/sqrt(2)));
// }

TEST_CASE("Get wheel velocities", "[wheel velocities]") // Sarah, Ziselman
{
    using namespace rigid2d;
    double wheelbase = 2;
    double wheelradius = 1;

    DiffDrive robot = DiffDrive(wheelbase, wheelradius, 0.0, 0.0, 0.0, 0.0, 0.0);

    Twist2D twist;
    twist.dth = PI/2;
    twist.dx = 0.0;
    twist.dy = 0.0;

    wheelVel velocities = robot.convertTwist(twist);

    REQUIRE(velocities.uL == Approx(-PI/2));
    REQUIRE(velocities.uR == Approx(PI/2));
}

// TEST_CASE("Get wheel velocities 2", "[wheel velocities 2]") // Sarah, Ziselman
// {
//     using namespace rigid2d;
//     double wheelbase = 2;
//     double wheelradius = 1;

//     DiffDrive robot = DiffDrive(wheelbase, wheelradius, 0.0, 0.0, 0.0, 0.0, 0.0);

//     Twist2D twist;
//     twist.dth = 0.0;
//     twist.dx = 2.0;
//     twist.dy = 3.0;

//     wheelVel velocities = robot.convertTwist(twist);

//     REQUIRE(velocities.uL == Approx(2.0));
//     REQUIRE(velocities.uR == Approx(2.0));
// }

TEST_CASE("Get wheel velocities 3", "[wheel velocities 3]") // Sarah, Ziselman
{
    using namespace rigid2d;
    double wheelbase = 2;
    double wheelradius = 1;

    DiffDrive robot = DiffDrive(wheelbase, wheelradius, 0.0, 0.0, 0.0, 0.0, 0.0);

    Twist2D twist;
    twist.dth = PI/3;
    twist.dx = 1.5;
    twist.dy = 1.5;

    wheelVel velocities = robot.convertTwist(twist);

    REQUIRE(velocities.uL == Approx((-PI/3) + 1.5));
    REQUIRE(velocities.uR == Approx((PI/3) + 1.5));
}