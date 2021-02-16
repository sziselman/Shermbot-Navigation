#include <catch_ros/catch.hpp>
#include <rigid2d/rigid2d.hpp>
#include <rigid2d/diff_drive.hpp>

/// \brief testing function that updates configuration of the diff drive robot
TEST_CASE("Update robot configuration", "[update configuration]") // Lin, Liu
{
    using namespace rigid2d; 
    double wheelbase = 1;
    double wheelradius = 1;

    // move forwards 
    DiffDrive config1 = DiffDrive(wheelbase, wheelradius, 0.0, 0.0, 0.0, 0.0, 0.0);

    double leftWheelNew = 0.1;
    double rightWheelNew = 0.1;
    config1(leftWheelNew, rightWheelNew);

    // dd.updateConfig(newpose1);

    // RobotConfig currpose1 = dd.getConfig();
    REQUIRE(config1.getTh() == Approx(0));
    REQUIRE(config1.getX() == Approx(0.1));
    REQUIRE(config1.getY() == Approx(0));


    // move forwards 
    DiffDrive config2 = DiffDrive(wheelbase, wheelradius, config1.getX(), config1.getY(), config1.getTh(), config1.getThL(), config1.getThR());
    
    double leftNew2 = 1.7;
    double rightNew2 = 1.3;

    config2(leftNew2, rightNew2);

    // dd2.updateConfig(newpose2);

    REQUIRE(config2.getTh() == Approx(-0.4));
    REQUIRE(config2.getX() == Approx(1.5));
    REQUIRE(config2.getY() == Approx(0));

}