#include <iostream>
#include "rigid2d.hpp"


int main() {
    rigid2d::Vector2D v{};
    v.x = 1.0;
    v.y = 2.0;
    // std::cout << "Enter info:  ";
    // std::cin >> v;
    // std::cout << "v = " << v << std::endl;

    
    rigid2d::Transform2D trans2;
    // rigid2d::Transform2D trans();
    // rigid2d::Transform2D trans(3.14);
    rigid2d::Transform2D trans(v, 4);
    std::cout << "trans = " << trans << std::endl;

    
    // std::cout << trans << std::endl;

    // rigid2d::Vector2D v2{};
    // v2 = trans(v);

    std::cout << "Enter info:  ";
    std::cin >> trans2;
    std::cout << "trans2 = " << trans2 << std::endl;

    // trans *= trans;
    // std::cout << trans << std::endl;
    
    // std::cout << trans.x() << std::endl;
    // std::cout << trans.y() << std::endl;
    // std::cout << trans.theta() << std::endl;
    std::cout << trans*trans2 << std::endl;

    return 0;
}