#include <iostream>
#include <cmath>
#include "nuslam/kalman.hpp"

namespace kalman {
    void test() {
        arma::mat A(4, 5, arma::fill::randu);
        arma::mat B(4, 5, arma::fill::randu);
        
        std::cout << A*B.t() << std::endl;
    }
}