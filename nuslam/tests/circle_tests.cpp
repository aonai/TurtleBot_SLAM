#include <sstream>
#include <catch_ros/catch.hpp>
#include "nuslam/circle.hpp"

TEST_CASE( "Test Cluster", "[Cluster]" ) {

    SECTION("test case 1") {
        arma::vec x {1, 2, 5, 7, 9, 3};
        arma::vec y {7, 6, 8, 7, 5, 7};
        circle::Cluster test {x, y};
        test.fit_circle();
        REQUIRE(test.get_center_x() == Approx(4.615482).epsilon(1e-4));
        REQUIRE(test.get_center_y() == Approx(2.807354).epsilon(1e-4));
        REQUIRE(test.get_circle_radius() == Approx(4.8275).epsilon(1e-4));
    }

    SECTION("test case 2") {    
        arma::vec x {-1, -0.3, 0.3, 1};
        arma::vec y {0, -0.06, 0.1, 0};
        circle::Cluster test {x, y};
        test.fit_circle();
        REQUIRE(test.get_center_x() == Approx(0.4908357).epsilon(1e-4));
        REQUIRE(test.get_center_y() == Approx(-22.15212).epsilon(1e-4));
        REQUIRE(test.get_circle_radius() == Approx(22.17979).epsilon(1e-4));
    }
}
