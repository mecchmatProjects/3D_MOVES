#pragma once
#include "AStar3D.h"
#include <algorithm>
#include <limits>
#include <random>
#include <vector>

struct RRTResult {
    std::vector<Vector3> path;
    int nodes=0;
};
inline RRTResult planRRT(int nx,int ny,int nz,Obstacle mover,Vector3 start,
                         Vector3 goal,const std::vector<Obstacle>& obstacles,
                         double margin=0.02,int maxIterations=12000,
                         unsigned seed=42,double step=0.85) {
    struct Node {Vector3 p;int parent;};
    RRTResult result;
    mover.position=start;
    if(!shapeSegmentFree(mover,start,obstacles,margin))return result;
    mover.position=goal;
    if(!shapeSegmentFree(mover,goal,obstacles,margin))return result;
    mover.position=start;
    if(shapeSegmentFree(mover,goal,obstacles,margin)) {
        result.path={start,goal};result.nodes=2;return result;
    }
    std::vector<Node> tree{{start,-1}};
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> ux(0,double(nx-1)),
        uy(0,double(ny-1)),uz(0,double(nz-1)),prob(0,1);
    for(int i=0;i<maxIterations;++i) {
        Vector3 sample=prob(rng)<0.17?goal:Vector3{ux(rng),uy(rng),uz(rng)};
        int closest=0;
        double best=std::numeric_limits<double>::infinity();
        for(int j=0;j<int(tree.size());++j) {
            const double d=tree[j].p.distanceTo(sample);
            if(d<best){best=d;closest=j;}
        }
        if(best<1e-10)continue;
        Vector3 to=tree[closest].p+normalized(sample-tree[closest].p)*std::min(step,best);
        mover.position=tree[closest].p;
        if(!shapeSegmentFree(mover,to,obstacles,margin))continue;
        int index=int(tree.size());
        tree.push_back({to,closest});
        mover.position=to;
        if(to.distanceTo(goal)<=step*1.5 &&
           shapeSegmentFree(mover,goal,obstacles,margin)) {
            tree.push_back({goal,index});
            std::vector<Vector3> reversed;
            for(int j=int(tree.size())-1;j>=0;j=tree[j].parent)
                reversed.push_back(tree[j].p);
            std::reverse(reversed.begin(),reversed.end());
            result.path=std::move(reversed);
            result.nodes=int(tree.size());
            return result;
        }
    }
    result.nodes=int(tree.size());
    return result;
}
