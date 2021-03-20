#ifndef CIRCLE_GUARD_HPP
#define CIRCLE_GUARD_HPP

#include <armadillo>

namespace circle {

    /// \brief threshold for groupoing clusters. If distance to current and previous
    /// points have different < group_thresh, then current point belongs to the same cluster.
    double group_thresh = 0.05;

    /// \brief group points into different clusters
    /// \param ranges - laser scan distance message
    /// \param angles - angle corresponding to each range in ranges
    /// \return an arma of vector, where each vector represent a cluster
    arma::field <arma::vec> group_clusters(arma::vec ranges, arma::vec angles);

    class Cluster {
        /// \brief A cluster of points.
        private:
            arma::vec x_arr;
            arma::vec y_arr;
            arma::vec z_arr;
            arma::vec ang_arr;
            bool is_circle = false;
            double center_x, center_y, circle_radius;
            double is_circle_thresh = 1;

        public:
            /// \brief Create a cluster from x and y positions
            /// \param x - x position of points
            /// \param y - y position of points
            Cluster(arma::vec x, arma::vec y);
            
            /// \brief Create a cluster from measurements
            /// \param z - measurements
            explicit Cluster(arma::vec z);

            /// \brief Create an empty cluster
            Cluster();

            /// \brief Check whether the cluster represent a circle
            /// is_circle = true if yes; otherwise false.
            /// If the cluster is a circle, radius will be stored in circle_radius.
            /// Cirlce's x y positions will be stored in center_x and center_y.
            void fit_circle();

            /// \brief Check whether the cluster is a circle or wall
            /// This is to limit false classification from fit_circle()
            /// Modify is_circle to false if the cluster is a wall. 
            void classify_arc();

            /// \brief get whether the cluster is a circle
            /// \return true is is a circle; otherwise, false.
            bool check_is_circle();

            /// \brief get x position of circle
            /// \return x of circle
            double get_center_x();
            
            /// \brief get y position of circle
            /// \return get y of circle
            double get_center_y();

            /// \brief get radius of circle
            /// \return get radius of circle
            double get_circle_radius();

                        
    };

};

#endif