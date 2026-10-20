#include <opencv2/core.hpp>
#include "PuReST.h"
extern "C" {
void* purest_create() { return new PuReST(); }
void purest_destroy(void* pointer) { delete static_cast<PuReST*>(pointer); }
void purest_reset(void* pointer) { static_cast<PuReST*>(pointer)->reset(); }
int purest_run(void* pointer, unsigned char* data, int width, int height,
               int stride, int x, int y, int w, int h, float* result) {
    try {
        cv::Mat frame(height, width, CV_8UC1, data, stride);
        cv::Rect roi = cv::Rect(x,y,w,h) & cv::Rect(0,0,width,height);
        if (roi.width < 10 || roi.height < 10) return 0;
        Pupil pupil;
        static_cast<PuReST*>(pointer)->run(frame, roi, pupil);
        result[0]=pupil.center.x; result[1]=pupil.center.y;
        result[2]=pupil.confidence;
        return pupil.confidence >= .6f && pupil.size.width > 0;
    } catch (...) { return 0; }
}
}
