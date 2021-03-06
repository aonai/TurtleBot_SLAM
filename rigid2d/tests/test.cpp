#include <sstream>
#include <catch_ros/catch.hpp>
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"

TEST_CASE( "Test Vector2D struct and operators", "[Vector2D]" ) {
    std::stringstream ss;
    rigid2d::Vector2D v {1.1, 2.2};
    rigid2d::Vector2D v2 {3, 4};
    rigid2d::Vector2D v3 {};

    SECTION( "initialize Vector2D" ) {
        REQUIRE(v.x == 1.1);
        REQUIRE(v.y == 2.2);
    }

    SECTION( "output Vector2D" ) {
        ss << v;
        std::string str =  ss.str();
        std::string cmp ("[1.1 2.2]");
        REQUIRE( str == cmp );
    }

    SECTION( "input Vector2D" ) {
        ss.str("");
        ss << "1.2 3.4";
        ss >> v;
        REQUIRE(v.x == 1.2);
        REQUIRE(v.y == 3.4);
    }

    SECTION( "input Vector2D different form" ) {
        ss.str("");
        ss << "[1.2 3.4]";
        ss >> v;
        REQUIRE(v.x == 1.2);
        REQUIRE(v.y == 3.4);
    }

    SECTION( "add two Vector2D" ) {
        v += v2;
        REQUIRE(v.x == 4.1);
        REQUIRE(v.y == 6.2);

        v3 = v + v2;
        REQUIRE(v3.x == 7.1);
        REQUIRE(v3.y == 10.2);
    }

    SECTION( "subtract two Vector2D" ) {
        v2 -= v;
        REQUIRE(v2.x == 1.9);
        REQUIRE(v2.y == (4-2.2));

        v3 = v2 - v2;
        REQUIRE(v3.x == 0);
        REQUIRE(v3.y == 0);
    }

    SECTION( "multiply two Vector2D" ) {
        v *= v2;
        REQUIRE(v.x == (1.1*3+2.2*4));
        REQUIRE(v.y == 0);

        v3 = v * v2;
        REQUIRE(v3.x == ((1.1*3+2.2*4)*3));
        REQUIRE(v3.y == 0);
    }

    SECTION( "magnitude and angle of Vector2D" ) {
        double mag = rigid2d::magnitude(v);
        REQUIRE(mag == Approx(2.460).epsilon(0.01));

        double ang = rigid2d::angle(v);
        REQUIRE(ang == Approx(1.107).epsilon(0.01));
    }

    SECTION( "constructors of Vector2D" ) {
        rigid2d::Vector2D test_v;
        REQUIRE(test_v.x == 0.0);
        REQUIRE(test_v.y == 0.0);

        rigid2d::Vector2D test_v2(2, 3);
        REQUIRE(test_v2.x == 2);
        REQUIRE(test_v2.y == 3);
    }
    
}

TEST_CASE( "Test NormalVector2D helper function and operators", "[NormalVector2D]" ) {
    std::stringstream ss;
    rigid2d::Vector2D v {1.1, 2.2};
    rigid2d::NormalVector2D u{};
        
    SECTION( "normalize vector" ) {
        u  = normalize(v);
        REQUIRE(u.ux == Approx(0.45).epsilon(0.01));
        REQUIRE(u.uy == Approx(0.89).epsilon(0.01));
    }

    SECTION( "output Vector2D" ) {
        u.ux = 1.0;
        u.uy = 0.0;
        ss << u;
        std::string str =  ss.str();
        std::string cmp ("[1 0]");
        REQUIRE( str == cmp );
    }

}

TEST_CASE( "Test Twist2D struct and operators", "[Twist2D]" ) {
    std::stringstream ss;
    rigid2d::Twist2D t {1.1, 2.2, 3.3};

    SECTION( "initialize Twist2D" ) {
        REQUIRE(t.omg == 1.1);
        REQUIRE(t.vx == 2.2);
        REQUIRE(t.vy == 3.3);
    }

    SECTION( "output Twist2D" ) {
        ss << t;
        std::string str =  ss.str();
        std::string cmp ("[1.1 2.2 3.3]");
        REQUIRE( str == cmp );
    }

    SECTION( "input Twist2D" ) {
        ss.str("");
        ss << "1.2" << ' ' << "3.4" << ' ' << "5.6";
        ss >> t;
        REQUIRE(t.omg == 1.2);
        REQUIRE(t.vx == 3.4);
        REQUIRE(t.vy == 5.6);
    }

    SECTION( "input Twist2D different form" ) {
        ss.str("");
        ss << "[1.2 3.4 5.6]";
        ss >> t;
        REQUIRE(t.omg == 1.2);
        REQUIRE(t.vx == 3.4);
        REQUIRE(t.vy == 5.6);
    }
}

TEST_CASE( "Test Transform2D class", "[Transform2D]" ) {
    std::stringstream ss;

    SECTION( "input Transform2D and empty constructor") {
        rigid2d::Transform2D T;
        ss.str("");
        ss << "10 2.2 3.3";
        ss >> T;
        REQUIRE(T.theta() == 10);
        REQUIRE(T.x() == 2.2);
        REQUIRE(T.y() == 3.3);
    }

    SECTION( "output Transform2D and constructor with two arguments") {
        rigid2d::Transform2D T(rigid2d::Vector2D {1.1, 2.2}, 3.3);
        ss << T;
        std::string str =  ss.str();
        std::string cmp ("dtheta (degrees): 0.580513 dx: 1.1 dy: 2.2");
        REQUIRE( str == cmp );
    }

    SECTION( "constructor with input vector" ) {
        rigid2d::Transform2D T(rigid2d::Vector2D {1.1, 2.2});
        REQUIRE(T.theta() == 0);
        REQUIRE(T.x() == 1.1);
        REQUIRE(T.y() == 2.2);
    }

    SECTION( "constructor with input randian" ) {
        rigid2d::Transform2D T(1.1);
        REQUIRE(T.theta() == 1.1);
        REQUIRE(T.x() == 0);
        REQUIRE(T.y() == 0);
    }

    SECTION( "operator () with input vector" ) {
        rigid2d::Transform2D T(rigid2d::Vector2D {1, 2}, 3);
        rigid2d::Vector2D v {4, 5};
        rigid2d::Vector2D result {};
        result = T(v);
        REQUIRE(result.x == Approx(-3.67).epsilon(0.01));
        REQUIRE(result.y == Approx(-2.39).epsilon(0.01));
    }

    SECTION( "operator () with input twist" ) {
        rigid2d::Transform2D T(rigid2d::Vector2D {1, 2}, 3);
        rigid2d::Twist2D t {4, 5, 6};
        rigid2d::Twist2D result {};
        result = T(t);
        REQUIRE(result.omg == Approx(4).epsilon(0.01));
        REQUIRE(result.vx == Approx(2.20).epsilon(0.01));
        REQUIRE(result.vy == Approx(-9.23).epsilon(0.01));
    }

    SECTION ( "inverse function" ) {
        rigid2d::Transform2D T(rigid2d::Vector2D {1, 2}, 3);
        rigid2d::Transform2D result;
        result = T.inv();
        REQUIRE(result.theta() == Approx(-3).epsilon(0.01));
        REQUIRE(result.x() == Approx(0.71).epsilon(0.01));
        REQUIRE(result.y() == Approx(2.12).epsilon(0.01));
    }

    SECTION ( "operator T1*=T2" ) {
        rigid2d::Transform2D T1(rigid2d::Vector2D {1, 2}, 3);
        rigid2d::Transform2D T2(rigid2d::Vector2D {4, 5}, 6);
        T1 *= T2;
        REQUIRE(T1.theta() == Approx(2.72).epsilon(0.01));
        REQUIRE(T1.x() == Approx(-3.67).epsilon(0.01));
        REQUIRE(T1.y() == Approx(-2.39).epsilon(0.01));
    }

    SECTION ( "operator T3 = T1*T2" ) {
        rigid2d::Transform2D T1(rigid2d::Vector2D {1, 2}, 3);
        rigid2d::Transform2D T2(rigid2d::Vector2D {4, 5}, 6);
        rigid2d::Transform2D result;
        result = T1*T2;
        REQUIRE(result.theta() == Approx(2.72).epsilon(0.01));
        REQUIRE(result.x() == Approx(-3.67).epsilon(0.01));
        REQUIRE(result.y() == Approx(-2.39).epsilon(0.01));
    }

    SECTION ( "integrate twist function " ) {
        rigid2d::Transform2D T1(rigid2d::Vector2D {2, 3}, 1);
        rigid2d::Twist2D t{1, 2, 3};
        rigid2d::Transform2D result;

        result = T1.integrateTwist(t);
        REQUIRE(result.theta() == Approx(2.00).epsilon(0.01));
        REQUIRE(result.x() == Approx(-0.734).epsilon(0.01));
        REQUIRE(result.y() == Approx(5.12).epsilon(0.01));
    }

    SECTION ( "integrate twist function pure translation" ) {
        rigid2d::Transform2D T1(rigid2d::Vector2D {2, 3}, 1);
        rigid2d::Twist2D t{0, 2, 3};
        rigid2d::Transform2D result;

        result = T1.integrateTwist(t);
        REQUIRE(result.theta() == Approx(1).epsilon(0.01));
        REQUIRE(result.x() == Approx(4).epsilon(0.01));
        REQUIRE(result.y() == Approx(6).epsilon(0.01));
    }

    SECTION ( "integrate twist function pure rotation" ) {
        rigid2d::Transform2D T1(rigid2d::Vector2D {2, 3}, 1);
        rigid2d::Twist2D t{1, 0, 0};
        rigid2d::Transform2D result;

        result = T1.integrateTwist(t);
        REQUIRE(result.theta() == Approx(2).epsilon(0.01));
        REQUIRE(result.x() == Approx(2).epsilon(0.01));
        REQUIRE(result.y() == Approx(3).epsilon(0.01));
    }

    SECTION ( "convert angle" ) {
        double rad = 3.34;
        double result = rigid2d::normalize_angle(rad);
        REQUIRE(result == Approx(-2.94).epsilon(0.01));

        rad = -3.34;
        result = rigid2d::normalize_angle(rad);
        REQUIRE(result == Approx(2.94).epsilon(0.01));
    }
}

TEST_CASE( "Test DiffDrive class and associated struct DiffDriveVel", "[DiffDrive]" ) {
    SECTION("struct DiffDriveVel") {
        rigid2d::DiffDriveVel vel {1.1, 2.2};

        REQUIRE(vel.vL == 1.1);
        REQUIRE(vel.vR == 2.2);
    }

    SECTION("constructor and functions with pure rotation") {
        rigid2d::DiffDrive dd(1.0, 1.0);
        rigid2d::Transform2D config = dd.config();
        rigid2d::Twist2D t{1, 0, 0};
        rigid2d::DiffDriveVel vel = dd.vel_from_twist(t);
        REQUIRE(vel.vL == -1);
        REQUIRE(vel.vR == 1);

        dd.update(-1, 1);
        rigid2d::Transform2D updated = dd.config();

        REQUIRE(updated.theta() == Approx(1).epsilon(0.01));
        REQUIRE(updated.x() == 0);
        REQUIRE(updated.y() == 0);
    }

    SECTION("constructor and functions with vel pure translation") {
        rigid2d::DiffDrive dd(1.0, 1.0);
        rigid2d::Transform2D config = dd.config();
        rigid2d::Twist2D t{0, 1, 0};
        rigid2d::DiffDriveVel vel = dd.vel_from_twist(t);
        REQUIRE(vel.vL == 1);
        REQUIRE(vel.vR == 1);

        dd.update(1, 1);
        rigid2d::Transform2D updated = dd.config();

        REQUIRE(updated.theta() == 0);
        REQUIRE(updated.x() == 1);
        REQUIRE(updated.y() == 0);
    }

    SECTION("constructor and functions") {
        rigid2d::DiffDrive dd(1.0, 1.0);
        rigid2d::Transform2D config = dd.config();
        rigid2d::Twist2D t{1, 1, 0};
        rigid2d::DiffDriveVel vel = dd.vel_from_twist(t);
        REQUIRE(vel.vL == 0);
        REQUIRE(vel.vR == 2);

        dd.update(0, 2);
        rigid2d::Transform2D updated = dd.config();

        REQUIRE(updated.theta() == Approx(1).epsilon(0.01));
        REQUIRE(updated.x() == Approx(0.841).epsilon(0.01));
        REQUIRE(updated.y() == Approx(0.460).epsilon(0.01));
    }
}