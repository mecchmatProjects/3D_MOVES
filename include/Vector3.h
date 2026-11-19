#pragma once
#include <cmath>
struct Vector3 {
    double x=0,y=0,z=0;
    Vector3()=default;
    Vector3(double a,double b,double c):x(a),y(b),z(c){}
    Vector3 operator+(Vector3 b) const {return {x+b.x,y+b.y,z+b.z};}
    Vector3 operator-(Vector3 b) const {return {x-b.x,y-b.y,z-b.z};}
    Vector3 operator*(double k) const {return {x*k,y*k,z*k};}
    Vector3 operator/(double k) const {return {x/k,y/k,z/k};}
    Vector3& operator+=(Vector3 b){x+=b.x;y+=b.y;z+=b.z;return *this;}
    double length() const{return std::sqrt(x*x+y*y+z*z);}
    double distanceTo(Vector3 b) const{return (*this-b).length();}
};
inline double dot(Vector3 a,Vector3 b){return a.x*b.x+a.y*b.y+a.z*b.z;}
inline Vector3 normalized(Vector3 a){double l=a.length();return l>1e-12?a/l:Vector3{};}
