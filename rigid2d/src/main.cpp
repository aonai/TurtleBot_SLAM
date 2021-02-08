#include <iostream>
#include "../include/rigid2d/rigid2d.hpp"
#include "../include/rigid2d/diff_drive.hpp"

/**
int main() {

    rigid2d::Transform2D Tab;
    std::cout << "Enter Tab (dtheta dx dy):  ";
    std::cin >> Tab;

    rigid2d::Transform2D Tbc;
    std::cout << "Enter Tbc (dtheta dx dy):  ";
    std::cin >> Tbc;

    std::cout << "Tab = " << Tab << std::endl;
    std::cout << "Tbc = " << Tbc << std::endl;

    rigid2d::Transform2D Tba = Tab.inv();
    std::cout << "Tba = " << Tba << std::endl;

    rigid2d::Transform2D Tcb = Tbc.inv();
    std::cout << "Tcb = " << Tcb << std::endl;

    rigid2d::Transform2D Tac = Tab*Tbc;
    std::cout << "Tac = " << Tac << std::endl;

    rigid2d::Transform2D Tca = Tac.inv();
    std::cout << "Tca = " << Tca << std::endl;
    
    rigid2d::Vector2D v{};
    std::cout << "Enter vector [x y]:  ";
    std::cin >> v;

    char v_frame;
    std::cout << "Enter vector frame (a, b, or c):  ";
    std::cin >> v_frame;
    std::cout << "V" << v_frame << " = " << v << std::endl;

    rigid2d::NormalVector2D u{};
    u  = normalize(v);
    std::cout << "u = " << u  << std::endl;

    rigid2d::Twist2D t{};
    std::cout << "Enter twist [omg vx vy]:  ";
    std::cin >> t;
    std::cout << "Twist_" << v_frame << " = " << t << std::endl;
    std::cout << "trasformed twist = " << Tab(t) << std::endl;

    return 0;
}
**/

int main() {
    rigid2d::DiffDrive dd(1, 1);
    // rigid2d::Transform2D T = dd.config();
    
    // rigid2d::Twist2D t2{3.14/4, 1, 2};
    // std::cout << "after twist: " << T.integrateTwist(t2) << std::endl;

    // rigid2d::Twist2D t{0.3, 0.2, 0};
    // std::cout << "after twist: " << T.integrateTwist(t) << std::endl;
    // std::cout << "twist: " << T << std::endl;
    
    // rigid2d::DiffDriveVel vel = dd.vel_from_twist(t);
    // std::cout << "vel = " << vel.vL << " " << vel.vR << std::endl;

    // dd.update(0.1, 0.1, false);
    // std::cout << "update = " << dd.config() << std::endl;

    // dd.update(-0.1, 0.1, false);
    // std::cout << "update = " << dd.config() << std::endl;
    
    // // dd.update(1, -1, false);
    // // std::cout << "update = " << dd.config() << std::endl;

    // dd.update(0.1, 0.1, false);
    // std::cout << "update = " << dd.config() << std::endl;


    // dd.update(-0.2, 0.2, false);
    // std::cout << "update = " << dd.config() << std::endl;


    // dd.update(-0.1, 0.5, true);
    // std::cout << "update = " << dd.config() << std::endl;



    // rigid2d::Transform2D test (3.14/4);
    // rigid2d::Transform2D eye;
    // std::cout << "t*eye = " << test*eye << std::endl;

    // std::cout << rigid2d::normalize_angle(3.34) << std::endl;

}
