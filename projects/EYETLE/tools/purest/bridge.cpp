// ABI wrapper; upstream PuRe/PuReST sources retain their non-commercial license.
#include <opencv2/opencv.hpp>
#include <cmath>
#include <algorithm>
#include "PuReST.h"
extern "C" {
void* purest_create() {try {return new PuReST();} catch (...) {return nullptr;}}
void purest_destroy(void* h) {delete static_cast<PuReST*>(h);}
void purest_reset(void* h) {if(h) static_cast<PuReST*>(h)->reset();}
int purest_process(void* handle,const unsigned char* data,int width,int height,double* out) {
    if(!handle||!data||!out||width<32||height<32) return -1;
    std::fill(out,out+7,0.);
    try {
        cv::Mat frame(height,width,CV_8UC1,const_cast<unsigned char*>(data));
        Pupil p=static_cast<PuReST*>(handle)->runWithConfidence(frame);
        if(!p.valid()||!std::isfinite(p.outline_confidence)) return 0;
        double confidence=p.confidence>=0?std::min(p.confidence,p.outline_confidence):p.outline_confidence;
        out[0]=p.center.x;out[1]=p.center.y;out[2]=p.size.width;out[3]=p.size.height;out[4]=p.angle;out[5]=confidence;
        out[6]=std::all_of(out,out+6,[](double v){return std::isfinite(v);}) && confidence>=.6 &&
               p.center.x>2 && p.center.x<width-2 && p.center.y>2 && p.center.y<height-2;
        return 0;
    } catch (...) {static_cast<PuReST*>(handle)->reset();return -2;}
}
}
