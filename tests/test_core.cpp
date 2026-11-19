#include "AStar3D.h"
#include "Physics.h"
#include "RRT3D.h"
#include "Telemetry.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>
static bool close(double a,double b,double tol=1e-5){return std::abs(a-b)<tol;}
int main(){
    const auto acceleration=[](const State3D& s){return Vector3{0,-9.81,0}-s.velocity*0.25;};
    const State3D initial{{0,4,0},{2,4,0},{},0};
    auto e=initial,r=initial;
    for(int i=0;i<100;++i){e=Physics3D::euler(e,0.01,acceleration);r=Physics3D::rk4(r,0.01,acceleration);}
    assert(std::isfinite(e.position.y)&&std::isfinite(r.position.y));
    assert(e.position.distanceTo(r.position)>0.001);
    auto linear=[](const State3D&){return Vector3{2,-3,1};};
    const State3D q{{0,1,2},{1,1,1},{},0};
    auto ans=Physics3D::rk4(q,1.0,linear);
    assert(close(ans.position.x,2.0)&&close(ans.position.y,0.5)&&close(ans.position.z,3.5));
    assert(close(ans.velocity.x,3.0));
    const Obstacle sphere{ShapeType::Sphere,{0,0,0},{0.5,0,0}};
    const Obstacle box{ShapeType::Box,{2,0,0},{1.0,0.3,0.2},{0,0,45}};
    assert(separation(sphere,box)>0.0);
    const Obstacle near{ShapeType::Sphere,{1.35,0,0},{0.5,0,0}};
    assert(separation(near,box)<0.1);
    const Obstacle c{ShapeType::Cylinder,{2,0,0},{0.4,2,0},{90,0,0}};
    assert(std::isfinite(separation(c,box)));
    std::vector<Obstacle> obstacles{{ShapeType::Sphere,{3,2,1},{0.75,0,0}},
        {ShapeType::Box,{5,2,1},{0.6,0.8,0.35},{0,40,15}},
        {ShapeType::Cylinder,{7,2,1},{0.45,1.8,0},{30,0,0}}};
    Obstacle mover{ShapeType::Sphere,{0,2,1},{0.2,0,0}};
    for(auto alg:{SearchAlgorithm::AStar,SearchAlgorithm::Dijkstra}){
      for(auto criterion:{PathCriterion::Distance,PathCriterion::Time,PathCriterion::Combined}){
        AStar3D planner(10,7,6,mover,obstacles,0.02,2.0,1.5,criterion,1.0,alg);
        auto p=planner.findPathFrom(mover.position,GridPoint{9,2,1});
        assert(p.size()>=2);
        for(size_t i=1;i<p.size();++i){Obstacle m=mover;m.position=p[i-1];
            assert(shapeSegmentFree(m,p[i],obstacles,0.02));}
      }
    }
    {
      AStar3D fractional(10,7,6,mover,obstacles);
      auto trajectory=fractional.findPathFrom({0.2,2.1,1.1},Vector3{8.7,2.4,1.2});
      assert(!trajectory.empty());
      assert(trajectory.front().distanceTo({0.2,2.1,1.1})<1e-9);
      assert(trajectory.back().distanceTo({8.7,2.4,1.2})<1e-9);
    }
    auto result=planRRT(10,7,6,mover,mover.position,{9,2,1},obstacles,0.02,5000);
    assert(!result.path.empty());
    Telemetry telemetry;telemetry.setGoal({2,0,0});telemetry.newRun();
    telemetry.record(q,IntegrationMethod::Euler);telemetry.finishRun();
    assert(telemetry.runs().size()==1);
    std::cout<<"All core smoke tests passed\n";
}
