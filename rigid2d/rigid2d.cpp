#include <iostream>
#include <limits>
#include <cmath>
#include <cstring>
#include "rigid2d.hpp"

namespace rigid2d {

    std::ostream & operator<<(std::ostream & os, const Vector2D & v) 
    { 
        os << '[' << v.x << ' ' << v.y << ']';
        return os;
    }

    std::istream & operator>>(std::istream & is, Vector2D & v)
    {   
        bool ignore = false;
        if (is.peek()=='['){
            ignore = true; 
        }
        if (ignore) {
            is.ignore(std::numeric_limits<std::streamsize>::max(), '[');
            is >> v.x;
            is >> v.y;
            is.ignore(std::numeric_limits<std::streamsize>::max(), ']');
        }
        else {
            is >> v.x;
            is >> v.y;
        }

        return is;
    }

    Transform2D::Transform2D() {
    }
    
    Transform2D::Transform2D(const Vector2D & trans)  : tran(trans)
    {
        mat[0][2] = trans.x;
        mat[1][2] = trans.y;
    }

    Transform2D::Transform2D(double radians) : rad(radians)
    {
        deg = rad2deg(rad);
        mat[0][0] = cos(radians);
        mat[0][1] = -sin(radians);
        mat[1][0] = sin(radians);
        mat[1][1] = cos(radians);
    }

    Transform2D::Transform2D(const Vector2D & trans, double radians) : tran(trans), rad(radians) {
        deg = rad2deg(rad);
        mat[0][0] = cos(radians);
        mat[0][1] = -sin(radians);
        mat[1][0] = sin(radians);
        mat[1][1] = cos(radians);
        mat[0][2] = trans.x;
        mat[1][2] = trans.y;
    }

    Vector2D Transform2D::operator()(Vector2D v) const{
        Vector2D toReturn{};
        toReturn.x = mat[0][0]*v.x + mat[0][1]*v.y + mat[0][2];
        toReturn.y = mat[1][0]*v.x + mat[1][1]*v.y + mat[1][2];
        return toReturn;
    }

    Transform2D Transform2D::inv(void) const{
        Vector2D newV{};
        newV.x = -1*(mat[0][0]*tran.x + mat[1][0]*tran.y);
        newV.y = -1*(mat[0][1]*tran.x + mat[1][1]*tran.y);
        double newRad = rad*-1;
        Transform2D inverted(newV, newRad);
        return inverted;
    }

    Transform2D & Transform2D::operator*=(const Transform2D & rhs) {
        double multMat [3][3];
        for (int i = 0; i < 3; ++i)
        {
            multMat[i][0] = this->mat[i][0]*rhs.mat[0][0] + this->mat[i][1]*rhs.mat[1][0] + this->mat[i][2]*rhs.mat[2][0]; 
            multMat[i][1] = this->mat[i][0]*rhs.mat[0][1] + this->mat[i][1]*rhs.mat[1][1] + this->mat[i][2]*rhs.mat[2][1]; 
            multMat[i][2] = this->mat[i][0]*rhs.mat[0][2] + this->mat[i][1]*rhs.mat[1][2] + this->mat[i][2]*rhs.mat[2][2];
        }   

        Vector2D newV{multMat[0][2], multMat[1][2]};
        double cosRad = acos(multMat[0][0]);
        double sinRad = asin(multMat[1][0]);
        double newRad = cosRad;
        if (almost_equal(cosRad, -1*sinRad, 1e-2)) {
            newRad = cosRad*-1;
        }
        this->tran = newV;
        this->rad = newRad;
        this->deg = rad2deg(newRad);
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                this->mat[i][j] = multMat[i][j];
            }
        }
        return *this;
    }

    double Transform2D::x() const{
        return tran.x;
    }

    double Transform2D::y() const{
        return tran.y;
    }

    double Transform2D::theta() const{
        return rad;
    }

    std::ostream & operator<<(std::ostream & os, const Transform2D & tf) {
        os << "dtheta (degrees): " << tf.rad << " dx: " << tf.tran.x << " dy: " << tf.tran.y << std::endl;
        os << "mat = " << std::endl;
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                os << tf.mat[i][j] << ' ';
            }
            os << std::endl;
        }
        return os;
    }
    
    std::istream & operator>>(std::istream & is, Transform2D & tf){
        Vector2D newV{};
        double newRad;
        bool ignore = false;
        char str[6];
        if (is.peek()=='d'){
            is.get(str, 6);
            if (strcmp(str, "dtheta")){
                ignore = true; 
            }
        }
        if (ignore) {
            is.ignore(std::numeric_limits<std::streamsize>::max(), ':');
            is >> newRad;
            is.ignore(std::numeric_limits<std::streamsize>::max(), 'x');
            is.ignore(std::numeric_limits<std::streamsize>::max(), ':');
            is.peek();
            is >> newV.x;
            is.ignore(std::numeric_limits<std::streamsize>::max(), 'y');
            is.ignore(std::numeric_limits<std::streamsize>::max(), ':');
            is.peek();
            is >> newV.y;
        }
        else {
            is >> newRad;
            is >> newV.x;
            is >> newV.y;
        }
        Transform2D trans(newV, newRad);
        tf = trans;
        return is;
    }

    Transform2D operator*(Transform2D lhs, const Transform2D & rhs){
        return lhs*=rhs;
    }


}