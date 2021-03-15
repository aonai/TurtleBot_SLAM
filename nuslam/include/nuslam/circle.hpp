#ifndef CIRCLE_GUARD_HPP
#define CIRCLE_GUARD_HPP

#include <armadillo>

namespace circle {

    double group_thresh = 0.3;

    arma::field <arma::vec> group_clusters(arma::vec ranges, arma::vec angles);

    class Cluster {
        private:
            arma::vec x_arr;
            arma::vec y_arr;
            arma::vec z_arr;
            arma::vec ang_arr;
            bool is_circle = false;
            double center_x, center_y, circle_radius;
            double is_circle_thresh = 1;

        public:
            Cluster(arma::vec x, arma::vec y);
            
            explicit Cluster(arma::vec z);

            Cluster();

            void fit_circle();

            void classify_arc();

            bool check_is_circle();

            double get_center_x();
            
            double get_center_y();

            double get_circle_radius();

                        
    };

};

#endif