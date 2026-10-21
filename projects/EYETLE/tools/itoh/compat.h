#pragma once
#include <functional>
#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/core/types_c.h>
#include <singleeyefitter/math.h>
#include <singleeyefitter/solve.h>
#ifndef CV_AA
#define CV_AA cv::LINE_AA
#endif
using std::isnan;
