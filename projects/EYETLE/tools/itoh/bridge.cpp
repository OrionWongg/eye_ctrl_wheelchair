// Linux C ABI around Yuta Itoh's original pupil fitter and EyeModelUpdater.
#include "pupilFitter.h"
#include "eye_model_updater.h"
#include <memory>
#include <limits>
#include <algorithm>

struct ItohTracker {
    PupilFitter pupil;
    eye_tracker::EyeModelUpdater model;
    explicit ItohTracker(double focal):model(focal,5,.5) { pupil.setDebug(false); }
};

extern "C" {
void* itoh_create(double focal) {
    try { if (!std::isfinite(focal)||focal<=0) return nullptr; return new ItohTracker(focal); }
    catch (...) { return nullptr; }
}
void itoh_destroy(void* handle) { delete static_cast<ItohTracker*>(handle); }
void itoh_reset(void* handle) { if (handle) static_cast<ItohTracker*>(handle)->model.reset(); }

// Output layout: detected, built, valid, observations, cx,cy,w,h,angle,
// normal xyz, sphere xyz, pupil radius, reprojection similarity.
int itoh_observe(void* handle,const unsigned char* pixels,const double* ellipse,double* out) {
    if (!handle || !pixels || !ellipse || !out) return -1;
    std::fill(out,out+17,0.);
    auto& t=*static_cast<ItohTracker*>(handle);
    try {
        cv::Mat gray(480,640,CV_8UC1,const_cast<unsigned char*>(pixels));
        cv::RotatedRect rr(cv::Point2f(ellipse[0],ellipse[1]),cv::Size2f(ellipse[2],ellipse[3]),ellipse[4]);
        out[1]=t.model.is_model_built();out[3]=t.model.fitter_count();
        if (!std::all_of(ellipse,ellipse+5,[](double v){return std::isfinite(v);}) ||
            rr.size.width<2 || rr.size.height<2 || rr.size.width>300 || rr.size.height>300 ||
            rr.center.x<0 || rr.center.x>=640 || rr.center.y<0 || rr.center.y>=480) return 0;
        out[0]=1;std::copy(ellipse,ellipse+5,out+4);
        auto el=singleeyefitter::toEllipse<double>(eye_tracker::toImgCoordInv(rr,gray,1.));
        // Reject singular conics before storing them in the 30-sample model.
        auto candidates=singleeyefitter::unproject(el,1.,t.model.fitter().focal_length);
        if (!candidates.first.normal.allFinite() || !candidates.second.normal.allFinite()) return 0;
        std::vector<cv::Point2f> inliers;
        if (!t.model.is_model_built()) {
            cv::Mat retained=gray.clone();
            t.model.add_observation(retained,el,inliers);
        }
        out[1]=t.model.is_model_built();out[3]=t.model.fitter_count();
        if (t.model.is_model_built()) {
            auto circle=t.model.unproject(gray,el,inliers);
            const auto& sphere=t.model.fitter().eye;
            if (circle && circle.normal.allFinite() && sphere.centre.allFinite() &&
                std::isfinite(circle.radius) && circle.radius>0 && sphere.centre.z()>0) {
                out[16]=t.model.compute_reliability(gray,el,inliers);
                for (int i=0;i<3;i++) {out[9+i]=circle.normal[i];out[12+i]=sphere.centre[i];}
                out[15]=circle.radius;
                out[2]=std::isfinite(out[16]) && out[16]>.8;
            }
        }
        return 0;
    } catch (const std::exception& e) { std::cerr<<"ITOH: "<<e.what()<<std::endl; return -2; }
      catch (...) { return -3; }
}

int itoh_process(void* handle,const unsigned char* pixels,double* out) {
    if (!handle || !pixels || !out) return -1;
    std::fill(out,out+17,0.);
    auto& t=*static_cast<ItohTracker*>(handle);
    out[1]=t.model.is_model_built();out[3]=t.model.fitter_count();
    try {
        cv::Mat gray=cv::Mat(480,640,CV_8UC1,const_cast<unsigned char*>(pixels)).clone();
        cv::RotatedRect rr;std::vector<cv::Point2f> inliers;
        if (!t.pupil.pupilAreaFitRR(gray,rr,inliers)) return 0;
        double ellipse[5]={rr.center.x,rr.center.y,rr.size.width,rr.size.height,rr.angle};
        return itoh_observe(handle,pixels,ellipse,out);
    } catch (const std::exception& e) { std::cerr<<"ITOH pupil: "<<e.what()<<std::endl; return -2; }
      catch (...) {return -3;}
}
}
