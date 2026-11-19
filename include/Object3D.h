
#pragma once

#include "Vector3.h"

class Object3D {
public:
    Vector3 position;
    Vector3 velocity;
    double radius;

    Object3D(
        const Vector3& pos,
        const Vector3& vel,
        double r
    )
        : position(pos), velocity(vel), radius(r) {}

    void update(double deltaTime) {
        position = position + velocity * deltaTime;
    }
};
