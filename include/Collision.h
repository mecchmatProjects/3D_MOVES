#pragma once
#include "Shapes.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace CollisionDetail {
inline bool solve(int n, double a[5][6], double x[5]) {
    for(int k=0;k<n;++k) {
        int pivot=k;
        for(int i=k+1;i<n;++i) if(std::abs(a[i][k])>std::abs(a[pivot][k])) pivot=i;
        if(std::abs(a[pivot][k])<1e-13) return false;
        for(int j=k;j<=n;++j) std::swap(a[k][j],a[pivot][j]);
        const double v=a[k][k];
        for(int j=k;j<=n;++j) a[k][j]/=v;
        for(int i=0;i<n;++i) if(i!=k) {
            const double factor=a[i][k];
            for(int j=k;j<=n;++j) a[i][j]-=factor*a[k][j];
        }
    }
    for(int i=0;i<n;++i) x[i]=a[i][n];
    return true;
}
struct Simplex {std::array<Vector3,5> vertices{};int count=0;};
inline Vector3 closest(Simplex& s) {
    double best=std::numeric_limits<double>::infinity();
    int bestMask=0;
    Vector3 nearest{};
    std::array<Vector3,5> input=s.vertices;
    const int count=s.count;
    for(int mask=1;mask<(1<<count);++mask) {
        int index[5], k=0;
        for(int i=0;i<count;++i) if(mask&(1<<i)) index[k++]=i;
        double a[5][6]{}; double x[5]{};
        for(int i=0;i<k;++i) {
            for(int j=0;j<k;++j) a[i][j]=dot(input[index[i]], input[index[j]]);
            a[i][k]=1.0; a[i][k+1]=0.0;
        }
        for(int j=0;j<k;++j) a[k][j]=1.0;
        a[k][k]=0.0; a[k][k+1]=1.0;
        if(!solve(k+1,a,x)) continue;
        bool valid=true;
        for(int i=0;i<k;++i) if(x[i]<-1e-9) {valid=false;break;}
        if(!valid) continue;
        Vector3 p{};
        for(int i=0;i<k;++i) p+=input[index[i]]*x[i];
        const double d=dot(p,p);
        if(d<best) { best=d;nearest=p;bestMask=mask; }
    }
    if(bestMask==0) {
        int j=0;
        for(int i=1;i<count;++i) if(dot(input[i],input[i])<dot(input[j],input[j])) j=i;
        nearest=input[j];bestMask=1<<j;
    }
    s.count=0;
    for(int i=0;i<count;++i) if(bestMask&(1<<i)) s.vertices[s.count++]=input[i];
    return nearest;
}
inline Vector3 support(const Obstacle& a,const Obstacle& b,Vector3 d) {
    return shapeSupport(a,d)-shapeSupport(b,d*(-1.0));
}
}

inline double separation(const Obstacle& a,const Obstacle& b) {

    const double centers=a.position.distanceTo(b.position);
    if(a.type==ShapeType::Sphere && b.type==ShapeType::Sphere)
        return centers-a.size.x-b.size.x;
    const double sphereLower=centers-boundRadius(a)-boundRadius(b);
    if(sphereLower>2.05) return sphereLower;
    using namespace CollisionDetail;
    Simplex s;
    Vector3 dir=a.position-b.position;
    if(dir.length()<1e-12) dir={1,0,0};
    s.vertices[0]=support(a,b,dir);s.count=1;
    Vector3 nearest=s.vertices[0];
    for(int iter=0;iter<72;++iter) {
        const double d=nearest.length();
        if(d<1e-9) return 0.0;
        dir=nearest*(-1.0);
        const Vector3 next=support(a,b,dir);
        const double progress=dot(nearest,nearest)-dot(nearest,next);
        if(progress <= 1e-10*std::max(1.0,d)) return d;
        if(s.count>=4) {nearest=closest(s);if(s.count>=4)return nearest.length();}
        s.vertices[s.count++]=next;
        nearest=closest(s);
    }
    return nearest.length();
}
inline double sceneSeparation(const Obstacle& moving,const std::vector<Obstacle>& obstacles) {
    double best=std::numeric_limits<double>::infinity();
    for(const auto& o:obstacles) best=std::min(best,separation(moving,o));
    return best;
}
inline bool shapeSegmentFree(Obstacle moving,Vector3 to,
                             const std::vector<Obstacle>& obstacles,double margin=0.02) {
    if(obstacles.empty()) return true;
    const Vector3 start=moving.position, delta=to-start;
    const double length=delta.length();
    if(length<1e-13) return sceneSeparation(moving,obstacles)>margin+1e-7;
    double t=0.0;
    for(int i=0;i<100000;++i) {
        moving.position=start+delta*t;
        const double clear=sceneSeparation(moving,obstacles)-margin;
        if(clear<=1e-6) return false;
        if(t>=1.0) return true;

        const double increment=std::max(1e-8,0.45*clear/length);
        t=std::min(1.0,t+increment);
    }
    return false;
}
inline bool segmentFree(Vector3 a,Vector3 b,double radius,
                         const std::vector<Obstacle>& obstacles,double margin=0.02) {
    return shapeSegmentFree({ShapeType::Sphere,a,{radius,0,0}},b,obstacles,margin);
}
