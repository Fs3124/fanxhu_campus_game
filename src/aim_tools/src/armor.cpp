#include <cmath>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "types.hpp"

using namespace std;
using namespace cv;


namespace aim {
const cv::Scalar kRedLow(0, 120, 148), kRedHigh(11, 255, 255);
const cv::Scalar kRedWrapLow(172, 119, 150), kRedWrapHigh(178, 255, 255);
// const cv::Scalar kBlueLow(92, 117, 150), kBlueHigh(138, 255, 255);        //    hsv初值


constexpr int kMinimumLightHeightPx = 4, kMinimumLightAspect = 2;                           //     长方形相关
constexpr double kMinimumPairSpacing = 1.2, kMaximumPairSpacing = 6, kPairTolerance = 0.4;  //  拿来配对的



//     灯条配对    （三项几何判断

Target pair_lights(const Rect& left, const Rect& right) {
    Target target;

    double length = (left.height + right.height) / 2.0;
    double left_x = left.x + left.width / 2.0;  // 中心点横坐标
    double right_x = right.x + right.width / 2.0;
    double y_diff = abs(left.y + left.height / 2.0 - (right.y + right.height / 2.0));


    if (right_x - left_x < kMinimumPairSpacing * length || right_x - left_x > kMaximumPairSpacing * length)
        return target;  // 如果left反而在右边，也会被筛，反正那个循环可以遍历全部情况
    if (y_diff > kPairTolerance * length || abs(left.height - right.height) > kPairTolerance * length)
        return target;

    target.found = true;

    target.corners = {{(float)left_x, (float)left.y}, {(float)right_x, (float)(right.y)}, {(float)(right_x), (float)(right.y + right.height)}, {(float)left_x, (float)(left.y + left.height)}};

    target.center_px = cv::Point2f((left_x + right_x) / 2, (left.y + right.y + length) / 2);

    return target;
}


Target find_armor(cv::Mat& image, const std::string& color, cv::Point2f previous) {
    cv::Mat hsv, mask, red_high;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, kRedLow, kRedHigh, mask);
    cv::inRange(hsv, kRedWrapLow, kRedWrapHigh, red_high);

    cv::bitwise_or(mask, red_high, mask);  // HSV红色跨越色相首尾



    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);


    std::vector<cv::Rect> lights;
    for (const std::vector<cv::Point>& contour : contours) {
        cv::Rect bar = cv::boundingRect(contour);
        if (bar.height >= kMinimumLightHeightPx && bar.height > kMinimumLightAspect * bar.width)
            lights.push_back(bar);
    }
    Target best;


    double best_distance = 2 * pow(image.cols, 2);

    if (previous.x == 0)
        previous = Point2f(image.cols / 2.0, image.rows / 2.0);

    for (int left = 0; left < static_cast<int>(lights.size()); left++) {
        for (int right = 0; right < static_cast<int>(lights.size()); right++) {
            if (left == right)
                continue;

            Target candidate = pair_lights(lights[left], lights[right]);

            if (!candidate.found)
                continue;

            double distance = cv::norm(candidate.center_px - previous);
            if (distance < best_distance) {
                best = candidate;
                best_distance = distance;
            }
        }
    }

    return best;
}




//      把目标位置转到世界坐标系（避免自己转头也被算进速度里


void estimate_position(Target& target, const cv::Mat& camera, const cv::Mat& distortion, const State& state) {

    const double width_m = 0.135, height_m = 0.055;  //  目标尺寸 (米


    std::vector<cv::Point3d> object = {{-width_m / 2, -height_m / 2, 0}, {width_m / 2, -height_m / 2, 0}, {width_m / 2, height_m / 2, 0}, {-width_m / 2, height_m / 2, 0}};


    cv::Mat rotation, translation;

    bool pnp_success = cv::solvePnP(object, target.corners, camera, distortion, rotation, translation, false, cv::SOLVEPNP_IPPE);
    if (!pnp_success)
        target.found = false;

    if (!target.found)
        return;

    target.position_m = camera_to_world({translation.at<double>(0), translation.at<double>(1), translation.at<double>(2)}, state);
    target.found = translation.at<double>(2) > 0;
}

cv::Point3d camera_to_world(const cv::Point3d& point, const State& state) {  // 转换到绝对的世界坐标
    double camera_right_m = point.x, camera_down_m = point.y, camera_forward_m = point.z;
    double pitch_rad = state.pitch_deg * kPi / 180;
    double yaw_rad = state.yaw_deg * kPi / 180;
    double forward_m = cos(pitch_rad) * camera_forward_m - sin(pitch_rad) * camera_down_m;


    return {cos(yaw_rad) * forward_m - sin(yaw_rad) * camera_right_m + state.origin_m.x,
            sin(yaw_rad) * forward_m + cos(yaw_rad) * camera_right_m + state.origin_m.y,
            sin(pitch_rad) * camera_forward_m + cos(pitch_rad) * camera_down_m + state.origin_m.z};  // 返回世界的绝对坐标
}



//  两帧求速度  换目标或断档就忘掉旧速度

void Armor::update_tracking(Target& target, const State& state) {  // 算速度
    double elapsed_s = state.stamp_s - previous_stamp_s;

    const double smoothing = 0.4, maximum_jump_px = 100;  // 初值


    if (elapsed_s <= 0 || elapsed_s > kMaximumTrackingGapS || cv::norm(target.center_px - previous_center_px) > maximum_jump_px)
        velocity_m_s = cv::Point3d();
    else {
        velocity_m_s.x = (1 - smoothing) * velocity_m_s.x + smoothing * (target.position_m.x - previous_position_m.x) / elapsed_s;
        velocity_m_s.y = (1 - smoothing) * velocity_m_s.y + smoothing * (target.position_m.y - previous_position_m.y) / elapsed_s;
        velocity_m_s.z = (1 - smoothing) * velocity_m_s.z + smoothing * (target.position_m.z - previous_position_m.z) / elapsed_s;
    }
    target.velocity_m_s = velocity_m_s;


    previous_position_m = target.position_m;
    previous_center_px = target.center_px;
    previous_stamp_s = state.stamp_s;
}


// 输入世界位置 速度   输出世界角 低弧飞行时间
AimCommand solve_aim(const Target& target, const State& state) {
    AimCommand command;
    if (!target.found)
        return command;

    double forward_m = target.position_m.x - state.origin_m.x;
    double right_m = target.position_m.y - state.origin_m.y;
    double down_m = target.position_m.z - state.origin_m.z;
    double distance_m = sqrt(forward_m * forward_m + right_m * right_m + down_m * down_m);
    double flight_s = distance_m / state.speed_m_s;
    double horizon_s = flight_s + max(0.0, state.age_s) + kProcessingDelayS;

    forward_m += target.velocity_m_s.x * horizon_s;  // 预测 补偿
    right_m += target.velocity_m_s.y * horizon_s;
    down_m += target.velocity_m_s.z * horizon_s;


    double horizontal_m = sqrt(forward_m * forward_m + right_m * right_m);
    double speed_squared = state.speed_m_s * state.speed_m_s;

    // 弹道方程   判别式
    double discriminant = speed_squared * speed_squared - kGravity * (kGravity * horizontal_m * horizontal_m - 2 * down_m * speed_squared);

    if (horizontal_m <= 0 || discriminant < 0)  // 水平距离为0或者弹道方程无解
        return command;

    double tangent = (-speed_squared + std::sqrt(discriminant)) / (kGravity * horizontal_m);

    command.pitch_deg = std::atan2(tangent, 1.0) * 180 / kPi;
    command.yaw_deg = std::atan2(right_m, forward_m) * 180 / kPi;
    command.flight_s = horizontal_m * std::sqrt(1 + tangent * tangent) / state.speed_m_s;
    command.elevation_deg = std::atan2(down_m, horizontal_m) * 180 / kPi - command.pitch_deg;


    double yaw_error_deg = std::remainder(command.yaw_deg - state.yaw_deg, 360.0);
    double pitch_error_deg = command.pitch_deg - state.pitch_deg;


    command.error_deg = std::sqrt(yaw_error_deg * yaw_error_deg + pitch_error_deg * pitch_error_deg);
    command.position_m = target.position_m;
    command.valid = std::isfinite(command.pitch_deg) && std::isfinite(command.yaw_deg) && std::isfinite(command.flight_s);


    return command;
}


//    用一堆if来决策

GateResult decide_gate(const AimCommand& command, const State& state, int lost_frames, bool enable_fire, double alignment_deg) {

    if (!state.source_valid || !std::isfinite(state.age_s) || state.age_s < 0 || state.age_s > kMaximumAgeS) return {false, false, "stale_or_invalid_source"};
    if (state.health <= 0) return {false, false, "player_dead"};
    if (lost_frames > 0) return {false, false, "target_lost"};
    if (!command.valid) return {false, false, "invalid_solution"};
    if (std::abs(command.pitch_deg) > kMaximumPitchDeg) return {false, false, "pitch_out_of_range"};


    if (!enable_fire) return {true, false, "fire_disabled"};
    if (!state.measured_speed || state.weakened) return {true, false, "state_forbids_fire"};
    if (state.ammo_remaining <= 0) return {true, false, "no_ammo"};
    if (alignment_deg <= 0 || alignment_deg > 1.2 || command.error_deg >= alignment_deg) return {true, false, "not_aligned"};


    return {true, true, "ok"};
}


void Armor::reset() {
    previous_stamp_s = 0;  // 直接设为0,让下一次的elapsed变得很大，可以跳过错误的速度计算
    velocity_m_s = cv::Point3d();
    previous_center_px = cv::Point2f();
    lost_frames = 1;
}


// 装甲板识别总入口
AimCommand Armor::process(cv::Mat& image, const cv::Mat& camera, const cv::Mat& distortion, const State& state, const std::string& color) {
    Target target = find_armor(image, color, previous_center_px);

    if (target.found) {
        // 画检测框
        for (int i = 0; i < 4; i++)
            cv::line(image, target.corners[i], target.corners[(i + 1) % 4], cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
        cv::circle(image, target.center_px, 3, cv::Scalar(0, 255, 255), -1);


        estimate_position(target, camera, distortion, state);
    }


    if (!target.found) {
        reset();
        return AimCommand();
    }

    lost_frames = 0;
    update_tracking(target, state);




    return solve_aim(target, state);
}



}  // namespace aim
