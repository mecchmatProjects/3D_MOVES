#pragma once
#include "Collision.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

struct GridPoint {int x=0,y=0,z=0;};
inline Vector3 gridVector(GridPoint p) {return {double(p.x),double(p.y),double(p.z)};}
enum class PathCriterion {Distance=0,Time=1,Combined=2};
enum class SearchAlgorithm {AStar,Dijkstra};

class AStar3D {
    int nx_,ny_,nz_;
    Obstacle mover_;
    const std::vector<Obstacle>& obstacles_;
    double margin_,maxSpeed_,acceleration_,timeWeight_;
    PathCriterion criterion_;
    SearchAlgorithm algorithm_;
    mutable int visited_=0;
    static constexpr int DCOUNT=7;
    static constexpr std::array<GridPoint,6> steps_{{
        {1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}
    }};
    bool inside(GridPoint p) const {
        return p.x>=0 && p.y>=0 && p.z>=0 && p.x<nx_ && p.y<ny_ && p.z<nz_;
    }
    int id(GridPoint p) const {return p.x+nx_*(p.y+ny_*p.z);}
    GridPoint at(int cell) const {return {cell%nx_,(cell/nx_)%ny_,cell/(nx_*ny_)};}
    double cost(double length,double turn) const {
        const double t=length/maxSpeed_+0.20*maxSpeed_/acceleration_*turn;
        if(criterion_==PathCriterion::Time) return t;
        if(criterion_==PathCriterion::Combined) return length+timeWeight_*t;
        return length;
    }
    double heuristic(Vector3 p,Vector3 goal) const {
        return algorithm_==SearchAlgorithm::Dijkstra?0.0:cost(p.distanceTo(goal),0.0);
    }
    static double angle(int incoming,int outgoing) {
        if(incoming==6) return 0.0;
        const GridPoint a=steps_[incoming],b=steps_[outgoing];
        const int dotp=a.x*b.x+a.y*b.y+a.z*b.z;
        return dotp==1?0.0:dotp==-1?SHAPE_PI:SHAPE_PI*0.5;
    }
    bool free(Vector3 from,Vector3 to) const {
        Obstacle object=mover_;object.position=from;
        return shapeSegmentFree(object,to,obstacles_,margin_);
    }
public:
    AStar3D(int nx,int ny,int nz,const Obstacle& mover,
            const std::vector<Obstacle>& obstacles,double margin=0.02,
            double maxSpeed=2.0,double acceleration=1.5,
            PathCriterion criterion=PathCriterion::Distance,double timeWeight=1.0,
            SearchAlgorithm algorithm=SearchAlgorithm::AStar)
        :nx_(nx),ny_(ny),nz_(nz),mover_(mover),obstacles_(obstacles),
         margin_(std::max(0.0,margin)),maxSpeed_(std::max(0.01,maxSpeed)),
         acceleration_(std::max(0.01,acceleration)),timeWeight_(std::max(0.0,timeWeight)),
         criterion_(criterion),algorithm_(algorithm) {}
    AStar3D(int nx,int ny,int nz,double radius,
            const std::vector<Obstacle>& obstacles,double margin=0.02)
        :AStar3D(nx,ny,nz,{ShapeType::Sphere,{}, {radius,0,0}},obstacles,margin) {}
    int visitedNodes() const {return visited_;}
    std::vector<Vector3> findPathFrom(Vector3 start,Vector3 target) const {
        visited_=0;
        if(nx_<=0 || ny_<=0 || nz_<=0 ||
           target.x<0.0 || target.y<0.0 || target.z<0.0 ||
           target.x>nx_-1 || target.y>ny_-1 || target.z>nz_-1 ||
           !free(start,start) || !free(target,target)) return {};
        if(free(start,target)) return {start,target};
        // The grid's six-neighbor topology produces optimal cost within the chosen
        // graph model. We connect a continuous starting point to nearby visible cells.
        const long long count64=1LL*nx_*ny_*nz_;
        if(count64>2000000) return {}; // Explicit cap for predictable memory use.
        const int count=int(count64);
        const double INF=std::numeric_limits<double>::infinity();
        std::vector<double> dist(std::size_t(count)*DCOUNT,INF);
        std::vector<int> parent(std::size_t(count)*DCOUNT,-2);
        std::vector<signed char> occupied(std::size_t(count),-1);
        std::vector<signed char> edge(std::size_t(count)*3,-1);
        using Entry=std::pair<double,int>;
        std::priority_queue<Entry,std::vector<Entry>,std::greater<Entry>> pq;
        auto vertexFree=[&](int cell) -> bool {
            signed char& v=occupied[cell];
            if(v<0) v=free(gridVector(at(cell)),gridVector(at(cell)))?1:0;
            return v==1;
        };
        auto edgeFree=[&](int cell,int dir,int other) -> bool {
            const int positive=(dir%2==0)?cell:other;
            const int axis=dir/2;
            signed char& e=edge[std::size_t(positive)*3+axis];
            if(e<0) e=free(gridVector(at(cell)),gridVector(at(other)))?1:0;
            return e==1;
        };
        bool seeded=false;
        const GridPoint nearestPoint{int(std::round(start.x)),int(std::round(start.y)),int(std::round(start.z))};
        for(int radius=1;radius<=6 && !seeded;++radius) {
            for(int z=std::max(0,nearestPoint.z-radius);z<=std::min(nz_-1,nearestPoint.z+radius);++z)
            for(int y=std::max(0,nearestPoint.y-radius);y<=std::min(ny_-1,nearestPoint.y+radius);++y)
            for(int x=std::max(0,nearestPoint.x-radius);x<=std::min(nx_-1,nearestPoint.x+radius);++x) {
                const int cell=id({x,y,z});
                const Vector3 p=gridVector({x,y,z});
                if(!vertexFree(cell) || !free(start,p))continue;
                const int state=cell*DCOUNT+6;
                const double g=cost(start.distanceTo(p),0.0);
                if(g<dist[state]) {
                    dist[state]=g;parent[state]=-1;
                    pq.push({g+heuristic(p,target),state});
                    seeded=true;
                }
            }
        }
        if(!seeded)return {};
        int bestTerminal=-1;
        double bestTerminalCost=INF;
        while(!pq.empty()) {
            if(pq.top().first >= bestTerminalCost-1e-9) break;
            const auto [f,state]=pq.top();pq.pop();
            const int cell=state/DCOUNT,incoming=state%DCOUNT;
            const GridPoint here=at(cell);
            const Vector3 from=gridVector(here);
            if(f>dist[state]+heuristic(from,target)+1e-9)continue;
            ++visited_;
            // Link any visible grid vertex to the continuous goal. This allows
            // arbitrary fractional coordinates without rounding the target.
            if(free(from,target)) {
                const Vector3 finalLeg=target-from;
                double turn=0.0;
                if(incoming!=6 && finalLeg.length()>1e-10) {
                    const auto dv=steps_[incoming];
                    const Vector3 axis{double(dv.x),double(dv.y),double(dv.z)};
                    turn=std::acos(std::clamp(dot(axis,normalized(finalLeg)),-1.0,1.0));
                }
                const double candidate=dist[state]+cost(finalLeg.length(),turn);
                if(candidate < bestTerminalCost) {
                    bestTerminalCost=candidate;
                    bestTerminal=state;
                }
            }
            for(int dir=0;dir<6;++dir) {
                const auto d=steps_[dir];
                const GridPoint n{here.x+d.x,here.y+d.y,here.z+d.z};
                if(!inside(n))continue;
                const int other=id(n);
                if(!vertexFree(other) || !edgeFree(cell,dir,other))continue;
                const int ns=other*DCOUNT+dir;
                const double g=dist[state]+cost(1.0,angle(incoming,dir));
                if(g+1e-9<dist[ns]) {
                    dist[ns]=g;parent[ns]=state;
                    pq.push({g+heuristic(gridVector(n),target),ns});
                }
            }
        }
        if(bestTerminal>=0) {
            std::vector<Vector3> path;
            for(int atState=bestTerminal;atState>=0;atState=parent[atState])
                path.push_back(gridVector(at(atState/DCOUNT)));
            std::reverse(path.begin(),path.end());
            if(path.empty() || path.front().distanceTo(start)>1e-8)
                path.insert(path.begin(),start);
            if(path.back().distanceTo(target)>1e-8)path.push_back(target);
            return path;
        }
        return {};
    }
    std::vector<Vector3> findPathFrom(Vector3 start,GridPoint goal) const {
        return findPathFrom(start,gridVector(goal));
    }
    std::vector<GridPoint> findPath(GridPoint start,GridPoint goal) const {
        const auto path=findPathFrom(gridVector(start),goal);
        std::vector<GridPoint> out;out.reserve(path.size());
        for(const auto p:path)out.push_back({int(std::round(p.x)),int(std::round(p.y)),int(std::round(p.z))});
        return out;
    }
};
inline std::vector<Vector3> smoothPath(const std::vector<Vector3>& raw,
                                      const Obstacle& mover,const std::vector<Obstacle>& obstacles,
                                      double margin=0.02) {
    if(raw.size()<3)return raw;
    std::vector<Vector3> result{raw.front()};
    std::size_t i=0;
    while(i+1<raw.size()) {
        std::size_t j=raw.size()-1;
        for(;j>i+1;--j) {
            Obstacle m=mover;m.position=raw[i];
            if(shapeSegmentFree(m,raw[j],obstacles,margin))break;
        }
        result.push_back(raw[j]);i=j;
    }
    return result;
}
