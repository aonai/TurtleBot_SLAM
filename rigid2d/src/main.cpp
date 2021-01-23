#include <iostream>
#include "../include/rigid2d/rigid2d.hpp"


int main() {

    rigid2d::Transform2D Tab;
    std::cout << "Enter Tab (dtheta dx dy):  ";
    std::cin >> Tab;

    rigid2d::Transform2D Tbc;
    std::cout << "Enter Tbc (dtheta dx dy):  ";
    std::cin >> Tbc;

    std::cout << "Tab = " << Tab << std::endl;
    std::cout << "Tbc = " << Tbc << std::endl;

    // rigid2d::Transform2D Tba = Tab.inv();
    // std::cout << "Tba = " << Tba << std::endl;

    // rigid2d::Transform2D Tcb = Tbc.inv();
    // std::cout << "Tcb = " << Tcb << std::endl;

    rigid2d::Transform2D Tac = Tab*Tbc;
    std::cout << "Tac = " << Tac << std::endl;

    // rigid2d::Transform2D Tca = Tac.inv();
    // std::cout << "Tca = " << Tca << std::endl;
    
    // rigid2d::Vector2D v{};
    // std::cout << "Enter vector [x y]:  ";
    // std::cin >> v;

    // char v_frame;
    // std::cout << "Enter vector frame (a, b, or c):  ";
    // std::cin >> v_frame;
    // std::cout << "V" << v_frame << " = " << v << std::endl;

    // rigid2d::NormalVector2D u{};
    // u  = normalize(v);
    // std::cout << "u = " << u  << std::endl;

    // rigid2d::Twist2D t{};
    // std::cout << "Enter twist [omg vx vy]:  ";
    // std::cin >> t;
    // std::cout << "Twist_" << v_frame << " = " << t << std::endl;
    // std::cout << "trasformed twist = " << Tab(t) << std::endl;

    return 0;
}