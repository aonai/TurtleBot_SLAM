#ifndef CIRCLE_GUARD_HPP
#define CIRCLE_GUARD_HPP

#include <armadillo>
#include "rigid2d/rigid2d.hpp"

namespace circle {

    double group_thresh = 0.1;

    arma::field <arma::vec> group_clusters(arma::vec ranges);

    class Cluster {
        private:
            arma::vec x_arr;
            arma::vec y_arr;
            arma::vec ranges;

        public:
            Cluster(arma::vec x, arma::vec y);
            
            Cluster(arma::vec r);
    };

};

#endif