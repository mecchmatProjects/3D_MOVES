#pragma once
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include "Telemetry.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

class AnalysisWindow {
    HWND window_=nullptr;
    HWND prev_=nullptr,next_=nullptr,all_=nullptr;
    const Telemetry* telemetry_=nullptr;
    int selectedRun_=-1; // -1 = all recorded runs
    inline static AnalysisWindow* instance_=nullptr;
    static constexpr int ID_PREV=8301,ID_NEXT=8302,ID_ALL=8303;

    static LRESULT CALLBACK windowProc(HWND hwnd,UINT msg,WPARAM w,LPARAM l) {
        AnalysisWindow* self=instance_;
        switch(msg) {
            case WM_CLOSE: ShowWindow(hwnd,SW_HIDE);return 0;
            case WM_COMMAND:
                if(self) {
                    const int id=LOWORD(w);
                    const int latest=self->telemetry_?self->telemetry_->run():0;
                    if(id==ID_ALL) self->selectedRun_=-1;
                    else if(id==ID_PREV) {
                        if(self->selectedRun_==-1) self->selectedRun_=latest;
                        self->selectedRun_=std::max(1,self->selectedRun_-1);
                    } else if(id==ID_NEXT) {
                        if(self->selectedRun_==-1) self->selectedRun_=1;
                        self->selectedRun_=std::min(latest,self->selectedRun_+1);
                    } else break;
                    InvalidateRect(hwnd,nullptr,TRUE);
                    return 0;
                }
                break;
            case WM_PAINT: {
                PAINTSTRUCT ps{};
                HDC dc=BeginPaint(hwnd,&ps);
                RECT rect{};GetClientRect(hwnd,&rect);
                if(self) self->paint(dc,rect);
                EndPaint(hwnd,&ps);
                return 0;
            }
        }
        return DefWindowProcA(hwnd,msg,w,l);
    }
    static void text(HDC dc,int x,int y,const std::string& v,COLORREF color=RGB(25,30,45)) {
        SetTextColor(dc,color);SetBkMode(dc,TRANSPARENT);
        TextOutA(dc,x,y,v.c_str(),int(v.size()));
    }
    static void drawPolyline(HDC dc,const std::vector<POINT>& pts,COLORREF color) {
        if(pts.size()<2) return;
        HPEN pen=CreatePen(PS_SOLID,2,color);
        HGDIOBJ old=SelectObject(dc,pen);
        Polyline(dc,pts.data(),int(pts.size()));
        SelectObject(dc,old);DeleteObject(pen);
    }
    void chart(HDC dc,RECT region,int mode) const {
        const char* names[3]={"Position X/Y/Z", "Speed", "Acceleration"};
        const int left=region.left+50,right=region.right-15;
        const int top=region.top+32,bottom=region.bottom-30;
        text(dc,region.left+12,region.top+5,names[mode]);
        if(right<=left || bottom<=top || !telemetry_) return;
        const auto& data=telemetry_->samples();
        std::vector<const MotionSample*> selected;
        for(const auto& p:data) if(selectedRun_<0 || p.run==selectedRun_)
            selected.push_back(&p);
        HPEN frame=CreatePen(PS_SOLID,1,RGB(165,172,188));
        HGDIOBJ old=SelectObject(dc,frame);
        MoveToEx(dc,left,top,nullptr);LineTo(dc,left,bottom);
        LineTo(dc,right,bottom);
        SelectObject(dc,old);DeleteObject(frame);
        if(selected.empty()) {text(dc,left+12,top+20,"No samples");return;}
        double xmax=0.0,ymin=1e100,ymax=-1e100;
        auto value=[&](const MotionSample& p,int component)->double {
            if(mode==0) return component==0?p.x:component==1?p.y:p.z;
            if(mode==1) return std::sqrt(p.vx*p.vx+p.vy*p.vy+p.vz*p.vz);
            return std::sqrt(p.ax*p.ax+p.ay*p.ay+p.az*p.az);
        };
        const int seriesCount=mode==0?3:1;
        for(const auto* p:selected) {
            xmax=std::max(xmax,p->t);
            for(int c=0;c<seriesCount;++c) {
                ymin=std::min(ymin,value(*p,c));
                ymax=std::max(ymax,value(*p,c));
            }
        }
        xmax=std::max(0.1,xmax);
        if(ymax-ymin<1e-7) {ymax+=0.5;ymin-=0.5;}
        for(int c=0;c<seriesCount;++c) {
            const COLORREF colors[3]={RGB(202,52,62),RGB(42,142,80),RGB(38,94,190)};
            std::vector<POINT> pts;
            pts.reserve(selected.size());
            int prevRun=-1;
            for(const auto* p:selected) {
                if(prevRun>=0 && prevRun!=p->run) {
                    drawPolyline(dc,pts,colors[c]);pts.clear();
                }
                prevRun=p->run;
                POINT point{};
                point.x=LONG(left+(right-left)*p->t/xmax);
                point.y=LONG(bottom-(bottom-top)*(value(*p,c)-ymin)/(ymax-ymin));
                pts.push_back(point);
            }
            drawPolyline(dc,pts,colors[c]);
        }
        char range[96];
        std::snprintf(range,sizeof(range),"max %.3f",ymax);text(dc,left+3,top+3,range);
        std::snprintf(range,sizeof(range),"min %.3f",ymin);text(dc,left+3,bottom-18,range);
        std::snprintf(range,sizeof(range),"Time 0 - %.2f s",xmax);text(dc,right-145,bottom+7,range);
    }
    void paint(HDC dc,RECT rect) const {
        FillRect(dc,&rect,(HBRUSH)GetStockObject(WHITE_BRUSH));
        std::string label=selectedRun_<0?"All experiments":"Run "+std::to_string(selectedRun_);
        text(dc,320,12,"Euler vs RK4 / "+label);
        const int chartTop=52;
        const int chartHeight=std::max(110,(int(rect.bottom)-chartTop-12)/3);
        for(int i=0;i<3;++i) {
            RECT r{10,LONG(chartTop+i*chartHeight),rect.right-10,
                    LONG(chartTop+(i+1)*chartHeight-6)};
            chart(dc,r,i);
        }
    }
public:
    bool create(HINSTANCE instance,const Telemetry* telemetry) {
        telemetry_=telemetry;instance_=this;
        WNDCLASSA wc{};
        wc.lpfnWndProc=windowProc;
        wc.hInstance=instance;
        wc.lpszClassName="ThreeDMovesAnalysisFinal";
        wc.hbrBackground=(HBRUSH)(COLOR_WINDOW+1);
        wc.hCursor=LoadCursor(nullptr,IDC_ARROW);
        RegisterClassA(&wc);
        window_=CreateWindowExA(0,wc.lpszClassName,"3D_MOVES - Analysis",
            WS_OVERLAPPEDWINDOW,100,100,850,850,nullptr,nullptr,instance,nullptr);
        if(!window_) return false;
        prev_=CreateWindowExA(0,"BUTTON","Previous run",WS_CHILD|WS_VISIBLE,
            12,10,100,28,window_,(HMENU)INT_PTR(ID_PREV),instance,nullptr);
        next_=CreateWindowExA(0,"BUTTON","Next run",WS_CHILD|WS_VISIBLE,
            120,10,100,28,window_,(HMENU)INT_PTR(ID_NEXT),instance,nullptr);
        all_=CreateWindowExA(0,"BUTTON","All runs",WS_CHILD|WS_VISIBLE,
            230,10,80,28,window_,(HMENU)INT_PTR(ID_ALL),instance,nullptr);
        return true;
    }
    void refresh() const {if(window_ && IsWindowVisible(window_)) InvalidateRect(window_,nullptr,FALSE);}
    void toggle() const {
        if(!window_) return;
        ShowWindow(window_,IsWindowVisible(window_)?SW_HIDE:SW_SHOW);
        if(IsWindowVisible(window_)) InvalidateRect(window_,nullptr,TRUE);
    }
};
