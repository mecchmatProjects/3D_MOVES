#pragma once
#include "Physics.h"
#include "Collision.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <string>
#include <vector>

enum class RunStatus {Running,Completed,Interrupted};
struct MotionSample {
    double t=0,x=0,y=0,z=0,vx=0,vy=0,vz=0,ax=0,ay=0,az=0;
    int method=0,run=0;
    double goalError=0.0, clearanceLowerBound=-1.0, specificKineticEnergy=0.0;
};
struct RunStatistics {
    int run=0,method=-1;
    RunStatus status=RunStatus::Running;
    double duration=0,distance=0,maxSpeed=0,maxAcceleration=0,endError=0;
    double minClearance=-1.0;
    std::size_t samples=0;
};
class Telemetry {
    std::vector<MotionSample> samples_;
    std::vector<RunStatistics> runs_;
    Vector3 goal_{};
    const std::vector<Obstacle>* environment_=nullptr;
    const Obstacle* mover_=nullptr;
    int run_=0;
    double nextSample_=0.0;
    static const char* statusName(RunStatus s) {
        switch(s) {
            case RunStatus::Running: return "Running";
            case RunStatus::Completed: return "Completed";
            case RunStatus::Interrupted: return "Interrupted";
        }
        return "Unknown";
    }
    void close(RunStatus status) {
        if(run_>0 && !runs_.empty() && runs_.back().status==RunStatus::Running)
            runs_.back().status=status;
    }
public:
    void setGoal(Vector3 v) {goal_=v;}
    void setEnvironment(const std::vector<Obstacle>* obstacles,const Obstacle* moving) {
        environment_=obstacles;mover_=moving;
    }
    Vector3 goal() const {return goal_;}
    void newRun() {
        close(RunStatus::Interrupted);
        ++run_;
        nextSample_=0.0;
        RunStatistics stats;stats.run=run_;
        runs_.push_back(stats);
    }
    void finishRun() {close(RunStatus::Completed);}
    void interruptRun() {close(RunStatus::Interrupted);}
    void clear() {samples_.clear();runs_.clear();run_=0;nextSample_=0;}
    int run() const {return run_;}
    const std::vector<MotionSample>& samples() const {return samples_;}
    const std::vector<RunStatistics>& runs() const {return runs_;}
    void record(const State3D& s,IntegrationMethod method) {
        if(run_<=0 || runs_.empty()) return;
        if(s.time+1e-9<nextSample_) return;
        MotionSample sm{s.time,s.position.x,s.position.y,s.position.z,
            s.velocity.x,s.velocity.y,s.velocity.z,
            s.acceleration.x,s.acceleration.y,s.acceleration.z,
            method==IntegrationMethod::Euler?0:1,run_};
        sm.goalError=s.position.distanceTo(goal_);
        sm.specificKineticEnergy=0.5*dot(s.velocity,s.velocity);
        if(environment_ && mover_ && !environment_->empty()) {
            Obstacle object=*mover_;
            object.position=s.position;
            sm.clearanceLowerBound=sceneSeparation(object,*environment_);
        }
        auto& r=runs_.back();
        r.method=sm.method;
        if(!samples_.empty() && samples_.back().run==run_) {
            const auto& prev=samples_.back();
            r.distance+=std::sqrt((sm.x-prev.x)*(sm.x-prev.x)+
                                  (sm.y-prev.y)*(sm.y-prev.y)+
                                  (sm.z-prev.z)*(sm.z-prev.z));
        }
        r.duration=sm.t;
        r.endError=sm.goalError;
        if(sm.clearanceLowerBound>=0.0 && (r.minClearance<0.0 || sm.clearanceLowerBound<r.minClearance))
            r.minClearance=sm.clearanceLowerBound;
        r.maxSpeed=std::max(r.maxSpeed,s.velocity.length());
        r.maxAcceleration=std::max(r.maxAcceleration,s.acceleration.length());
        ++r.samples;
        samples_.push_back(sm);
        nextSample_=s.time+0.04;
    }
    bool exportCSV(const std::string& filename) const {
        std::ofstream out(filename);
        if(!out) return false;
        out<<"run,method,time,x,y,z,vx,vy,vz,ax,ay,az,speed,acceleration,goal_error,clearance_lower_bound,specific_kinetic_energy\n";
        out<<std::fixed<<std::setprecision(7);
        for(const auto& s:samples_) {
            const double speed=std::sqrt(s.vx*s.vx+s.vy*s.vy+s.vz*s.vz);
            const double accel=std::sqrt(s.ax*s.ax+s.ay*s.ay+s.az*s.az);
            const double err=s.goalError;
            out<<s.run<<','<<(s.method==0?"Euler":"RK4")<<','<<s.t<<','
               <<s.x<<','<<s.y<<','<<s.z<<','<<s.vx<<','<<s.vy<<','<<s.vz<<','
               <<s.ax<<','<<s.ay<<','<<s.az<<','<<speed<<','<<accel<<','<<err<<','
               <<s.clearanceLowerBound<<','<<s.specificKineticEnergy<<'\n';
        }
        return bool(out);
    }
    bool exportSummaryCSV(const std::string& filename) const {
        std::ofstream out(filename);
        if(!out) return false;
        out<<"run,method,status,duration,travel_distance,max_speed,max_acceleration,final_goal_error,min_clearance_lower_bound,samples\n";
        out<<std::fixed<<std::setprecision(7);
        for(const auto& r:runs_) {
            out<<r.run<<','<<(r.method==0?"Euler":r.method==1?"RK4":"Unknown")<<','
               <<statusName(r.status)<<','<<r.duration<<','<<r.distance<<','
               <<r.maxSpeed<<','<<r.maxAcceleration<<','<<r.endError<<','
               <<r.minClearance<<','<<r.samples<<'\n';
        }
        return bool(out);
    }
    std::vector<MotionSample> samplesForRun(int run) const {
        std::vector<MotionSample> v;
        for(const auto& s:samples_) if(s.run==run)v.push_back(s);
        return v;
    }
};
