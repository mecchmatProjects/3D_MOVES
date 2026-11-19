#include "AStar3D.h"
#include "RRT3D.h"
#include "Physics.h"
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <vector>

using Clock=std::chrono::steady_clock;
static double length(const std::vector<Vector3>& path){
    double v=0;for(std::size_t i=1;i<path.size();++i)v+=path[i].distanceTo(path[i-1]);
    return v;
}
static double ms(Clock::time_point a){return std::chrono::duration<double,std::milli>(Clock::now()-a).count();}
static std::vector<Obstacle> scenario(int n,int nx,int ny,int nz) {
    // Deterministic; keep start and goal clear. A mix of boxes, cylinders and spheres.
    std::mt19937 rng(78211);
    std::uniform_real_distribution<double> ux(2.0,std::max(2.1,double(nx-3))),
        uy(0.5,std::max(0.6,double(ny-2))),
        uz(0.5,std::max(0.6,double(nz-2)));
    std::vector<Obstacle> obs;
    obs.reserve(n);
    for(int i=0;i<n;++i) {
        Obstacle o;
        o.type=static_cast<ShapeType>(i%3);
        o.position=i==0?Vector3{double(nx)*0.5,2,1}:
            Vector3{ux(rng),uy(rng),uz(rng)};
        if(o.type==ShapeType::Sphere)o.size={i==0?0.85:0.20,0,0};
        else if(o.type==ShapeType::Box)o.size={0.28,0.25,0.22};
        else o.size={0.24,0.55,0};
        o.rotation={double(i%43),double(i%47),double(i%59)};
        obs.push_back(o);
    }
    return obs;
}
static void benchmarkIntegration(std::ofstream& out) {
    std::cout<<"Integration dt study...\n";
    for(double h:{0.1,0.01,0.001}) {
        for(int m=0;m<2;++m) {
            const auto accel=[](const State3D&){return Vector3{0,-3,2};};
            State3D s{{0,1,0},{1,2,3},{},0};
            const int steps=int(std::round(1.0/h));
            auto start=Clock::now();
            for(int j=0;j<steps;++j)
                s=m==0?Physics3D::euler(s,h,accel):Physics3D::rk4(s,h,accel);
            const double elapsed=ms(start);
            const Vector3 exact{1,1+2-1.5,3+1};
            const double err=s.position.distanceTo(exact);
            out<<(m==0?"Euler":"RK4")<<','<<h<<','<<steps<<','<<elapsed<<','<<err<<'\n';
        }
    }
}
static void benchmarkPlanning(std::ofstream& out,bool full) {
    std::cout<<"Planning: comparing A*, Dijkstra, RRT; this can take a while in full mode.\n";
    const std::vector<int> obstacleCounts=full?std::vector<int>{0,10,50,100,500,1000}
        :std::vector<int>{0,10,50};
    for(int n:obstacleCounts) {
        constexpr int NX=20,NY=20,NZ=20;
        auto obs=scenario(n,NX,NY,NZ);
        Obstacle mover{ShapeType::Sphere,{0,2,1},{0.15,0,0}};
        const GridPoint goal{19,2,1};
        for(int a=0;a<3;++a) {
            const auto began=Clock::now();
            int visited=0;std::vector<Vector3> path;
            if(a==2) {
                const auto result=planRRT(NX,NY,NZ,mover,mover.position,gridVector(goal),
                                          obs,0.02,full?12000:3000,123);
                path=result.path;visited=result.nodes;
            } else {
                AStar3D planner(NX,NY,NZ,mover,obs,0.02,2,1.5,PathCriterion::Distance,1,
                    a==0?SearchAlgorithm::AStar:SearchAlgorithm::Dijkstra);
                path=planner.findPathFrom(mover.position,goal);
                visited=planner.visitedNodes();
            }
            out<<"obstacles,"<<n<<",20,20,20,"
               <<(a==0?"AStar":a==1?"Dijkstra":"RRT")
               <<",Distance,"<<visited<<','<<ms(began)<<','
               <<length(path)<<','<<(!path.empty())<<','
               <<((a==2)?std::size_t(visited)*40:
                   std::size_t(NX)*NY*NZ*88)<<'\n';
        }
    }
    for(int size:(full?std::vector<int>{20,50,100}:std::vector<int>{10,20})) {
        Obstacle mover{ShapeType::Sphere,{0,2,1},{0.15,0,0}};
        auto obs=scenario(full?30:5,size,size,size);
        const GridPoint goal{size-1,2,1};
        for(int c=0;c<3;++c) {
            AStar3D planner(size,size,size,mover,obs,0.02,2,1.5,
                static_cast<PathCriterion>(c));
            const auto began=Clock::now();
            const auto path=planner.findPathFrom(mover.position,goal);
            out<<"grid,"<<obs.size()<<','<<size<<','<<size<<','<<size
               <<",AStar,"<<(c==0?"Distance":c==1?"Time":"Combined")
               <<','<<planner.visitedNodes()<<','<<ms(began)<<','
               <<length(path)<<','<<(!path.empty())<<','
               <<std::size_t(size)*size*size*88<<'\n';
        }
    }
}
static void benchmarkDrag(std::ofstream& out) {
    std::cout<<"Drag model accuracy vs fine RK4 reference...\n";
    const auto acceleration=[](const State3D& s){
        return Vector3{0,-9.81,0}-s.velocity*0.25;
    };
    const State3D initial{{0,4,0},{1,2,0},{},0};
    State3D reference=initial;
    for(int i=0;i<10000;++i)reference=Physics3D::rk4(reference,0.0001,acceleration);
    for(double h:{0.1,0.01,0.001}) for(int mode=0;mode<2;++mode) {
        State3D state=initial;
        const auto began=Clock::now();
        const int steps=int(std::round(1.0/h));
        for(int i=0;i<steps;++i) {
            state=mode==0?Physics3D::euler(state,h,acceleration)
                :Physics3D::rk4(state,h,acceleration);
        }
        out<<(mode==0?"Euler":"RK4")<<','<<h<<','<<steps<<','
           <<state.position.distanceTo(reference.position)<<','
           <<state.velocity.distanceTo(reference.velocity)<<','<<ms(began)<<'\n';
    }
}
int main(int argc,char** argv) {
    const bool full=argc>=2 && std::string(argv[1])=="--full";
    std::ofstream integral("3d_moves_integration_experiments.csv");
    std::ofstream planning("3d_moves_planning_experiments.csv");
    std::ofstream drag("3d_moves_drag_accuracy.csv");
    if(!integral || !planning || !drag){std::cerr<<"Cannot open output CSV files\n";return 1;}
    integral<<"method,dt,steps,elapsed_ms,position_error\n";
    planning<<"experiment,obstacles,nx,ny,nz,algorithm,criterion,visited,elapsed_ms,path_length,path_found,estimated_memory_bytes\n";
    drag<<"method,dt,steps,position_error,velocity_error,elapsed_ms\n";
    integral<<std::setprecision(12);planning<<std::setprecision(12);
    benchmarkIntegration(integral);
    benchmarkDrag(drag);
    benchmarkPlanning(planning,full);
    std::cout<<"CSV created. For all README sizes, run with --full.\n";
    return 0;
}
