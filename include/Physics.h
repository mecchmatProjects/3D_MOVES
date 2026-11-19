#pragma once
#include "Vector3.h"
#include <algorithm>
#include <cmath>

struct State3D {
    Vector3 position{},velocity{},acceleration{};
    double time=0.0;
};
enum class IntegrationMethod {Euler,RK4};
struct Physics {
    double maxSpeed=2.0;
    double acceleration=1.5;
    double braking=2.0;
    double response=3.0;
    double maxTurnRate=180.0;
};
class Physics3D {
public:
    template<class Accel>
    static State3D euler(const State3D& s,double h,Accel f) {
        const Vector3 a=f(s);
        return {s.position+s.velocity*h,s.velocity+a*h,a,s.time+h};
    }
    template<class Accel>
    static State3D rk4(const State3D& s,double h,Accel f) {
        const Vector3 k1p=s.velocity, k1v=f(s);
        const State3D s2{s.position+k1p*(h*0.5),s.velocity+k1v*(h*0.5),{},s.time+h*0.5};
        const Vector3 k2p=s2.velocity, k2v=f(s2);
        const State3D s3{s.position+k2p*(h*0.5),s.velocity+k2v*(h*0.5),{},s.time+h*0.5};
        const Vector3 k3p=s3.velocity, k3v=f(s3);
        const State3D s4{s.position+k3p*h,s.velocity+k3v*h,{},s.time+h};
        const Vector3 k4p=s4.velocity, k4v=f(s4);
        State3D out;
        out.position=s.position+(k1p+k2p*2.0+k3p*2.0+k4p)*(h/6.0);
        out.velocity=s.velocity+(k1v+k2v*2.0+k3v*2.0+k4v)*(h/6.0);
        out.time=s.time+h;
        out.acceleration=f(out);
        return out;
    }
};
