#include <catch_ros/catch.hpp>
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"
#include <cmath>
#include <iostream>
#include <sstream>

/* !!!
Compile using command: g++ -Wall -Wextra -g -std=c++17 -o test test.cpp rigid2d.cpp (takes a really long time)
!!! 

Contributors:
Nahtaniel Nyberg
Sarah Ziselman
Lin Liu
*/ 

TEST_CASE("Default constructor creates identity transform","[identity_constructor]"){ // Sarah, Ziselman
    using namespace rigid2d;

    Transform2D identity = Transform2D();
        
    REQUIRE(almost_equal(identity.getCosTh(),1));
    REQUIRE(almost_equal(identity.getSinTh(),0));
    REQUIRE(almost_equal(identity.getX(),0));
    REQUIRE(almost_equal(identity.getY(),0));
}

TEST_CASE("Constructor using only radians component","[rad_constructor]"){ // Nathaniel, Nyberg
    using namespace rigid2d;

    Transform2D trans = Transform2D(PI);
        
    REQUIRE(almost_equal(trans.getCosTh(),cos(PI)));
    REQUIRE(almost_equal(trans.getSinTh(),sin(PI)));
    REQUIRE(almost_equal(trans.getX(),0));
    REQUIRE(almost_equal(trans.getY(),0));
}

TEST_CASE("Constructor using only vector component","[vector_constructor]"){ // Nathaniel, Nyberg
    using namespace rigid2d;

    Vector2D v;
    v.x = 2;
    v.y = 3;
    Transform2D trans = Transform2D(v);
        
    REQUIRE(almost_equal(trans.getCosTh(),1));
    REQUIRE(almost_equal(trans.getSinTh(),0));
    REQUIRE(almost_equal(trans.getX(),2));
    REQUIRE(almost_equal(trans.getY(),3));
}

/// \brief testing the 
TEST_CASE("Constructor using both components","[full_constructor]"){ // Nathaniel, Nyberg
    using namespace rigid2d;

    Vector2D v;
    v.x = 2;
    v.y = 3;
    Transform2D trans = Transform2D(v,PI);
        
    REQUIRE(almost_equal(trans.getCosTh(),cos(PI)));
    REQUIRE(almost_equal(trans.getSinTh(),sin(PI)));
    REQUIRE(almost_equal(trans.getX(),2));
    REQUIRE(almost_equal(trans.getY(),3));
}

/// \brief testing the istream operator
TEST_CASE("Input a Transform from istream","[input]"){ // Nathaniel, Nyberg
    using namespace rigid2d;

    Transform2D trans;

    std::unique_ptr<std::istream> is;
    is = std::make_unique<std::istringstream>(std::istringstream{"-30 2 3"});
    *is >> trans;

    REQUIRE(almost_equal(trans.getCosTh(),cos(-30)));
    REQUIRE(almost_equal(trans.getSinTh(),sin(-30)));
    REQUIRE(almost_equal(trans.getX(),2));
    REQUIRE(almost_equal(trans.getY(),3));
}

TEST_CASE("Outpt a Transform to ostream","[ouput]"){ // Nathaniel, Nyberg
    using namespace rigid2d;

    Transform2D trans = Transform2D();
    double test;
    std::stringstream ss;
    ss << trans;

    ss >> test;
    while(ss.fail()){
        ss.clear();
        ss.ignore(1);
        ss >> test;
    }

    REQUIRE(almost_equal(test,0));

    ss >> test;
    while(ss.fail()){
        ss.clear();
        ss.ignore(1);
        ss >> test;
    }

    REQUIRE(almost_equal(test,0));

    ss >> test;
    while(ss.fail()){
        ss.clear();
        ss.ignore(1);
        ss >> test;
    }

    REQUIRE(almost_equal(test,0));

}

/// \brief testing inv method for Transform2D
TEST_CASE("Inverse of Transform Matrix", "[transform]"){ // Sarah, Ziselman
    using namespace rigid2d;

    Vector2D v;
    v.x = 1;
    v.y = 2;

    Transform2D transMat = Transform2D(v, PI);
    Transform2D invTransMat = transMat.inv();

    REQUIRE(almost_equal(invTransMat.getCosTh(), cos(PI)));
    REQUIRE(almost_equal(invTransMat.getSinTh(), -sin(PI)));
    REQUIRE(almost_equal(invTransMat.getX(), 1));
    REQUIRE(almost_equal(invTransMat.getY(), 2));
}

/// \brief testing Vector2D operator()(Vector2D v) const
TEST_CASE("Transformation applied to Vector2D", "[transform]") // Lin, Liu 
{
    using namespace rigid2d; 
    Vector2D v; 
    v.x = -1;
    v.y = 2;

    Transform2D trans21 = Transform2D(v,PI); 

    Vector2D v1; 
    v1.x = -3; 
    v1.y = 1; 

    Vector2D v2; 
    v2 = trans21(v1);

    REQUIRE(almost_equal(v2.x, 2)); 
    REQUIRE(almost_equal(v2.y, 1));
}

TEST_CASE("Transform Composition Operator (*=)", "[transform]"){ // Arun, Kumar
    using namespace rigid2d;

    Vector2D v;
    v.x = 3;
    v.y = 9;

    Transform2D transMat = Transform2D(v, PI/9);
    Transform2D invTransMat = transMat.inv();

    transMat*=invTransMat;

    REQUIRE(almost_equal(transMat.getCosTh(), 1));
    REQUIRE(almost_equal(transMat.getSinTh(), 0));
    REQUIRE(almost_equal(transMat.getX(), 0));
    REQUIRE(almost_equal(transMat.getY(), 0));
}

TEST_CASE("Change twist reference frame", "[transform]"){ // Sarah, Ziselman
	using namespace rigid2d;
	
	Vector2D p;
	p.x = 1;
	p.y = 2;
	
	Transform2D transMat = Transform2D(p, PI/2);
	
	Twist2D tw = Twist2D();
    tw.dth = 2;
    tw.dx = 3;
    tw.dy = 5;

	Twist2D twNew = transMat(tw);
	REQUIRE(almost_equal(twNew.dth, 2));
	REQUIRE(almost_equal(twNew.dx, -1));
	REQUIRE(almost_equal(twNew.dy, 1));
}

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