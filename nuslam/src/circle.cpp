#include <iostream>
#include <cmath>
#include "nuslam/circle.hpp"

namespace circle {


    arma::field <arma::vec> group_clusters(arma::vec ranges){
        arma::field <arma::vec> clusters(100);
        int count = 0;
        int group_num = 0;
        arma::vec group (ranges.size());
        
        //group clusters 
        for (unsigned i = 1; i < ranges.size(); i++) {
            group(count) = ranges(i-1);
            double dist = fabs(ranges(i)-ranges(i-1));
            // std::cout << "i = " << i << " " << dist << std::endl;

            if (dist > group_thresh || i == ranges.size()-1) {
                if (i == ranges.size()-1 && dist <= group_thresh) {
                    count += 1;
                    group(count) = ranges(i);
                }
                group.resize(count+1);
                // std::cout << group_num << " group = " << group.t() << std::endl;
                clusters(group_num) = group;
                group_num += 1;
                count = 0;
                group = arma::vec (ranges.size());
            }
            else {
                count += 1;
            }
        }

        //check first and last cluster 
        arma::vec first = clusters(0);
        arma::vec last = clusters(group_num-1);
        double dist = fabs(first(0) - last(last.size()-1));
        arma::vec tmp (first.size() + last.size());
        if (dist <= group_thresh) {
            for (unsigned i = 0; i < tmp.size(); i++) {
                if (i < last.size()) {
                    tmp(i) = last(i);
                }
                else {
                    tmp(i) = first(i-last.size());
                }
            }
            // std::cout << group_num << " tmp = " << tmp.t() << std::endl;
            group_num -= 1;
            clusters(0) = tmp;
        }

        // resize clusters to return
        arma::field <arma::vec> toReturn (group_num);
        for (unsigned i = 0; i < group_num; i++) {
            toReturn(i) = clusters(i);
        }
        return toReturn;
    }

    Cluster::Cluster(arma::vec x, arma::vec y) : 
        x_arr(x),
        y_arr(y)
    {
        std::cout << "x = " << x_arr.t() << std::endl;
        std::cout << "y = " << y_arr.t() << std::endl;
    }

    Cluster::Cluster(arma::vec r) : ranges(r) {
        std::cout << "ranges = " << ranges.t() << std::endl;

    }


}
