#include "Collision.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <random>
static double exactSphereBox(const Obstacle& s,const Obstacle& b) {
    Vector3 p=rotateWorldToLocal(s.position-b.position,b.rotation);
    const auto clamp=[](double x,double h){return std::clamp(x,-h,h);};
    Vector3 closest{clamp(p.x,b.size.x),clamp(p.y,b.size.y),clamp(p.z,b.size.z)};
    return std::max(0.0,(p-closest).length()-s.size.x);
}
int main(){
    std::mt19937 rng(812);
    std::uniform_real_distribution<double> pos(-3,3),rot(-180,180),sz(.15,1.0);
    double err=0;int mismatches=0,checks=0;
    for(int i=0;i<3000;++i) {
        Obstacle sphere{ShapeType::Sphere,{pos(rng),pos(rng),pos(rng)},{sz(rng),0,0}};
        Obstacle box{ShapeType::Box,{pos(rng),pos(rng),pos(rng)},
            {sz(rng),sz(rng),sz(rng)},{rot(rng),rot(rng),rot(rng)}};
        const double exact=exactSphereBox(sphere,box);
        const double approx=separation(sphere,box);
        if(sphere.position.distanceTo(box.position)-boundRadius(sphere)-boundRadius(box)>2.05)
            continue;
        ++checks;
        err=std::max(err,std::abs(exact-approx));
        if(std::abs(exact-approx)>2e-4)++mismatches;
    }
    std::cout<<"Rotation sphere/OBB tested "<<checks<<" pairs, max difference "<<err<<", mismatches "<<mismatches<<"\n";
    assert(mismatches==0);
    const Obstacle mover{ShapeType::Sphere,{0,0,0},{0.15,0,0}};
    const std::vector<Obstacle> obs{{ShapeType::Box,{2,0,0},{0.2,1,1},{30,30,40}}};
    assert(!shapeSegmentFree(mover,{4,0,0},obs,0.02));
    assert(shapeSegmentFree(mover,{0,4,0},obs,0.02));
}
