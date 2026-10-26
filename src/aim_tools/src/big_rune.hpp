#pragma once
#include <deque>

#include "small_rune.hpp"

namespace aim {

class BigRunePipeline {
   public:
    explicit BigRunePipeline(RuneDetector& shared_detector) : detector(shared_detector) {}
    AimCommand process(cv::Mat& image, const cv::Mat& camera, const cv::Mat& distortion, const State& state);
    void reset();

    bool prediction_ready = false;
    int lost_frames = 1, unhit_targets = 0;
    std::size_t fit_samples = 0;
    double speed_rad_s = 0, prediction_horizon_s = 0;
    std::string reason = "waiting_for_big_rune";



   private:
    RuneDetector& detector;
    std::deque<cv::Point2d> samples;  // x是时间秒，y是连续转角弧度
    RunePose previous_pose;
    cv::Point2f previous_center_px;
    double previous_stamp_s = 0, phase_rad = 0, acceleration_rad_s2 = 0;
    void update_motion(RunePose& pose, double stamp_s);
};
}  // namespace aim
