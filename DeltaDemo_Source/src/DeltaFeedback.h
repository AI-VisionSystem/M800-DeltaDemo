#pragma once
#include "Settings.h"
#include <cmath>

namespace demo {
// Command-pulse feedback, not an encoder-measured TCP. Same axis convention as
// the existing M800 project: radial axes 0/120/240 degrees, positive arm down.
inline bool deltaPulsePose(const Settings& settings,const std::array<long,4>& pulses,
                           std::array<double,4>& angles,Pose& pose) {
    constexpr double pi=3.14159265358979323846,eps=1e-9;
    for(int i=0;i<4;++i){const auto& a=settings.axes[i];if(a.pulses<=0||a.angle<=0)return false;
        angles[i]=(a.deltaPol==0?1.0:-1.0)*double(pulses[i])*a.angle/a.pulses;
        if(!std::isfinite(angles[i]))return false;}
    const double base=settings.geometry[0],platform=settings.geometry[1];
    const double arm=settings.geometry[2],rod=settings.geometry[3];
    if(!std::isfinite(base)||!std::isfinite(platform)||!std::isfinite(arm)||!std::isfinite(rod)||base<=0||platform<=0||arm<=0||rod<=0)return false;
    std::array<std::array<double,3>,3> center;
    for(int i=0;i<3;++i){const double azimuth=i*2*pi/3,theta=angles[i]*pi/180;
        const double radius=base-platform+arm*std::cos(theta);
        center[i]={radius*std::cos(azimuth),radius*std::sin(azimuth),-arm*std::sin(theta)};}
    using Vec=std::array<double,3>;
    auto dot=[](const Vec& a,const Vec& b){return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];};
    Vec u,v;for(int k=0;k<3;++k){u[k]=center[1][k]-center[0][k];v[k]=center[2][k]-center[0][k];}
    const double d=std::sqrt(dot(u,u));if(d<eps)return false;
    Vec ex,ey,ez;for(int k=0;k<3;++k)ex[k]=u[k]/d;
    const double i=dot(ex,v);for(int k=0;k<3;++k)ey[k]=v[k]-i*ex[k];
    const double j=std::sqrt(dot(ey,ey));if(j<eps)return false;
    for(int k=0;k<3;++k)ey[k]/=j;
    ez={ex[1]*ey[2]-ex[2]*ey[1],ex[2]*ey[0]-ex[0]*ey[2],ex[0]*ey[1]-ex[1]*ey[0]};
    const double x=d/2,y=(i*i+j*j)/ (2*j)-i*x/j;
    const double z2=rod*rod-x*x-y*y;if(z2<0)return false;
    const double z=std::sqrt(z2)*(ez[2]>=0?-1:1); // Lower of the two sphere intersections.
    for(int k=0;k<3;++k){pose[k]=center[0][k]+x*ex[k]+y*ey[k]+z*ez[k];if(!std::isfinite(pose[k]))return false;}
    pose[3]=angles[3];return true;
}
}