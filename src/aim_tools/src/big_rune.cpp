#include "big_rune.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace aim {
void BigRunePipeline::reset() {

    samples.clear();

    previous_pose = RunePose();
    previous_center_px = cv::Point2f();
    previous_stamp_s = phase_rad = speed_rad_s = acceleration_rad_s2 = 0;
    prediction_horizon_s = 0;
    prediction_ready = false;
    lost_frames = 1;
    unhit_targets = 0;
    fit_samples = 0;


    reason = "waiting_for_big_rune";
}

void BigRunePipeline::update_motion(RunePose& pose, double stamp_s) {  // 获得速度 加速度
    prediction_ready = false;
    speed_rad_s = 0;
    acceleration_rad_s2 = 0;  // a

    //   比较相邻帧的半径向量  与小符一样，换板时去掉72度的整数倍
    if (previous_stamp_s > 0) {
        double elapsed = stamp_s - previous_stamp_s;

        if (elapsed <= 0 || elapsed > kMaximumTrackingGapS) {
            samples.clear();
            phase_rad = 0;


        } else {
            if (pose.normal.dot(previous_pose.normal) < 0)  // 防止法线反了
                pose.normal = -pose.normal;

            double angle = std::atan2(pose.normal.dot(previous_pose.radial_m.cross(pose.radial_m)), previous_pose.radial_m.dot(pose.radial_m));

            double step = std::remainder(angle, 2 * kPi / 5);

            if (std::isfinite(step) && std::abs(step / elapsed) <= 5) {
                phase_rad += step;
            } else {
                samples.clear();
                phase_rad = 0;
            }
        }
    }
    previous_pose = pose;
    previous_stamp_s = stamp_s;


    // 只保存最近0.6秒的数据
    samples.push_back(cv::Point2d(stamp_s, phase_rad));
    while (samples.size() > 1 && stamp_s - samples.front().x > 0.6)
        samples.pop_front();

    // 至少4个点，且覆盖了0.12秒
    fit_samples = samples.size();
    if (samples.size() < 4 || stamp_s - samples.front().x < 0.12)
        return;

    // 转角 = 当前角度 + 角速度*t + 0.5*角加速度*t**2
    // 当前帧的t为0
    cv::Mat matrix((int)(samples.size()), 3, CV_64F);
    cv::Mat angles((int)(samples.size()), 1, CV_64F);

    for (int i = 0; i < matrix.rows; ++i) {
        double t = samples[i].x - stamp_s;  // 填方程的系数
        matrix.at<double>(i, 0) = 1;
        matrix.at<double>(i, 1) = t;
        matrix.at<double>(i, 2) = 0.5 * t * t;


        angles.at<double>(i) = samples[i].y;
    }

    cv::Mat coefficients;  // 用于接收解

    if (!cv::solve(matrix, angles, coefficients, cv::DECOMP_SVD)) return;  // 用opencv的solve求解线性方程组   输入时间，累计转角  输出角速度， 角加速度的估计值
    if (!cv::checkRange(coefficients)) return;

    speed_rad_s = coefficients.at<double>(1);
    acceleration_rad_s2 = coefficients.at<double>(2);


    prediction_ready = std::abs(speed_rad_s) <= 5;
}



AimCommand BigRunePipeline::process(cv::Mat& image, const cv::Mat& camera, const cv::Mat& distortion, const State& state) {
    auto started = std::chrono::steady_clock::now();

    if (previous_stamp_s > 0 && state.stamp_s - previous_stamp_s > 0.5)
        reset();


    prediction_ready = false;
    lost_frames = 1;

    //   与小符使用同一个检测器
    auto targets = detector.detect(image);
    unhit_targets = (int)targets.size();

    if (targets.empty()) {
        reason = "big_rune_target_lost";
        return {};
    }


    //    首次选最高置信度，后选最靠近上次位置的板
    RuneDetection detection = targets.front();
    cv::Point2f selected_center;

    double best_distance = std::numeric_limits<double>::max();

    for (const auto& candidate : targets) {
        cv::Point2f center = (candidate.points[0] + candidate.points[1] + candidate.points[3] + candidate.points[4]) * 0.25;
        double distance = previous_stamp_s > 0 ? cv::norm(center - previous_center_px) : 1.0 - candidate.confidence;

        if (distance < best_distance) {
            best_distance = distance;
            detection = candidate;
            selected_center = center;
        }
    }

    RunePose pose = estimate_rune_pose(detection, camera, distortion, state);
    if (!pose.valid) {
        reason = "big_rune_pose_invalid";
        return {};
    }


    previous_center_px = selected_center;

    // 短时预测
    update_motion(pose, state.stamp_s);

    lost_frames = 0;
    double processing = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    double flight = cv::norm(pose.center_m + pose.radial_m - state.origin_m) / state.speed_m_s;


    // 与小符一样迭代三次  只不过大符多加一项角加速度
    AimCommand command;
    for (int i = 0; i < 3; ++i) {
        double horizon = flight + std::max(0.0, state.age_s) + processing + kProcessingDelayS;

        double speed;
        if (prediction_ready)
            speed = speed_rad_s + 0.5 * acceleration_rad_s2 * horizon;
        else
            speed = 0;

        Target target;
        target.found = true;

        target.position_m = predict_rune_position(pose, speed, horizon);
        command = solve_aim(target, state);  // 线速度默认0，避免重复添加提前量。

        prediction_horizon_s = horizon;  // 用于日志


        if (!command.valid)
            break;

        flight = command.flight_s;
    }
    reason = prediction_ready ? "big_rune_predicting" : "big_rune_warming";

    return command;
}
}  // namespace aim
