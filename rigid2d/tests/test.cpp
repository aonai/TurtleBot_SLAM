#define CATCH_CONFIG_MAIN 
#include <sstream>
#include "catch.hpp"
// #include "../src/rigid2d.cpp"
#include "../include/rigid2d/rigid2d.hpp"

TEST_CASE( "Test Vector2D struct and operators", "[Vector2D]" ) {
    std::stringstream ss;
    rigid2d::Vector2D v {1.1, 2.2};

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
        std::string cmp ("dtheta (degrees): 3.3 dx: 1.1 dy: 2.2");
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

}
