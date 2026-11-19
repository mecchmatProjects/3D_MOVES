#pragma once
#include "Vector3.h"
#include <cmath>

enum class ShapeType { Sphere, Box, Cylinder };
struct Obstacle {
    ShapeType type = ShapeType::Sphere;
    Vector3 position{3,2,1};
    Vector3 size{0.8,0.8,0.8};
    Vector3 rotation{0,0,0};
};
inline const char* shapeName(ShapeType t) {
    switch(t) {
        case ShapeType::Sphere: return "Sphere";
        case ShapeType::Box: return "Box";
        case ShapeType::Cylinder: return "Cylinder";
    }
    return "Unknown";
}
inline constexpr double SHAPE_PI=3.14159265358979323846;
inline Vector3 rotateLocalToWorld(Vector3 v, Vector3 r) {
    const double rx=r.x*SHAPE_PI/180.0, ry=r.y*SHAPE_PI/180.0, rz=r.z*SHAPE_PI/180.0;
    const double cx=std::cos(rx), sx=std::sin(rx);
    const double cy=std::cos(ry), sy=std::sin(ry);
    const double cz=std::cos(rz), sz=std::sin(rz);
    const Vector3 x{v.x, cx*v.y-sx*v.z, sx*v.y+cx*v.z};
    const Vector3 y{cy*x.x+sy*x.z, x.y, -sy*x.x+cy*x.z};
    return {cz*y.x-sz*y.y, sz*y.x+cz*y.y, y.z};
}
inline Vector3 rotateWorldToLocal(Vector3 v, Vector3 r) {
    const double rx=r.x*SHAPE_PI/180.0, ry=r.y*SHAPE_PI/180.0, rz=r.z*SHAPE_PI/180.0;
    const double cx=std::cos(rx), sx=std::sin(rx);
    const double cy=std::cos(ry), sy=std::sin(ry);
    const double cz=std::cos(rz), sz=std::sin(rz);
    const Vector3 z{cz*v.x+sz*v.y, -sz*v.x+cz*v.y, v.z};
    const Vector3 y{cy*z.x-sy*z.z, z.y, sy*z.x+cy*z.z};
    return {y.x, cx*y.y+sx*y.z, -sx*y.y+cx*y.z};
}
inline double boundRadius(const Obstacle& o) {
    if(o.type==ShapeType::Sphere) return o.size.x;
    if(o.type==ShapeType::Box) return o.size.length();
    return std::hypot(o.size.x,o.size.y*0.5);
}
inline Vector3 shapeSupport(const Obstacle& o, Vector3 direction) {
    if(o.type==ShapeType::Sphere)
        return o.position + normalized(direction)*o.size.x;
    const Vector3 d=rotateWorldToLocal(direction,o.rotation);
    Vector3 p;
    if(o.type==ShapeType::Box) {
        p={d.x>=0?o.size.x:-o.size.x,d.y>=0?o.size.y:-o.size.y,
           d.z>=0?o.size.z:-o.size.z};
    } else {
        const double rad=std::hypot(d.x,d.z);
        p={rad>1e-14?o.size.x*d.x/rad:o.size.x,
           d.y>=0?o.size.y*0.5:-o.size.y*0.5,
           rad>1e-14?o.size.x*d.z/rad:0.0};
    }
    return o.position + rotateLocalToWorld(p,o.rotation);
}
