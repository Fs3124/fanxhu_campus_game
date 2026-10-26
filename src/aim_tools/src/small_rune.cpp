#include "small_rune.hpp"

#include <openssl/evp.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <stdexcept>

using namespace std;

namespace aim {
bool RuneShotCooldown::allow(bool requested, int ammo, double now_s) {
    if (ammo < 0) return false;
    // 公开弹量减少才是出弹反馈；从收到反馈起再冷却0.5秒。
    if (previous_ammo >= 0 && ammo < previous_ammo)
        next_allowed_s = now_s + seconds;
    previous_ammo = ammo;
    if (!requested || now_s < next_allowed_s) return false;
    next_allowed_s = now_s + seconds;
    return true;
}

RuneDetector::RuneDetector(const std::string& path) {
    std::ifstream file(path, std::ios::binary);

    vector<unsigned char> bytes((std::istreambuf_iterator<char>(file)), {});
    unsigned char digest[EVP_MAX_MD_SIZE];
    unsigned int length = 0;

    detector = cv::dnn::readNetFromONNX(bytes);
    detector.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    detector.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    //  订阅前完成一次内核预热
    detect(cv::Mat(720, 960, CV_8UC3, cv::Scalar(114, 114, 114)));
}

std::vector<RuneDetection> decode_rune_targets(const cv::Mat& output, float scale, int px, int py, cv::Size size) {
    std::vector<RuneDetection> candidates;
    if (output.type() != CV_32F || output.dims != 3 || output.size[0] != 1 || output.size[1] != 18 || output.size[2] != 6300 || !output.isContinuous() || scale <= 0)
        return candidates;

    const float* values = output.ptr<float>();

    for (int anchor = 0; anchor < 6300; anchor++) {
        float score = values[anchor];  // 0待打击   1小符已击打  2大符已击打


        if (!std::isfinite(score) || !std::isfinite(values[6300 + anchor]) || !std::isfinite(values[6300 * 2 + anchor]) || score < 0.65 || score > 1 || score <= values[6300 + anchor] || score <= values[2 * 6300 + anchor])
            continue;

        RuneDetection candidate;
        candidate.found = true;
        candidate.confidence = score;

        //   五个点
        for (int p = 0; p < 5; p++) {

            int channel = 3 + p * 3;
            float x = (values[channel * 6300 + anchor] - px) / scale;  //  坐标按比例换算 （ /sacle
            float y = (values[(channel + 1) * 6300 + anchor] - py) / scale;
            float confidence = values[(channel + 2) * 6300 + anchor];


            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(confidence) || confidence < 0.50 || confidence > 1 || x < 0 || y < 0 || x >= size.width || y >= size.height) {
                candidate.found = false;  // 必须五个点都符合条件
            }

            candidate.points[p] = {x, y};
        }
        if (candidate.found)
            candidates.push_back(candidate);
    }
    std::stable_sort(candidates.begin(), candidates.end(), [](const auto& a, const auto& b) { return a.confidence > b.confidence; });
    std::vector<RuneDetection> targets;

    // 去重
    for (const auto& candidate : candidates) {

        bool duplicate = false;
        cv::Point2f center = (candidate.points[0] + candidate.points[1] + candidate.points[3] + candidate.points[4]) * 0.25;
        for (const auto& k : targets) {
            cv::Point2f other = (k.points[0] + k.points[1] + k.points[3] + k.points[4]) * 0.25;
            if (cv::norm(center - other) < 30) {
                duplicate = true;  //  靠中心点距离去重
                break;
            }
        }
        if (!duplicate)
            targets.push_back(candidate);
    }
    return targets;
}




std::vector<RuneDetection> RuneDetector::detect(const cv::Mat& image) {

    float scale = std::min(640.f / image.cols, 480.f / image.rows);
    int width = std::lround(image.cols * scale), height = std::lround(image.rows * scale);
    int px = (640 - width) / 2, py = (480 - height) / 2;


    cv::Mat resized, boxed;
    cv::resize(image, resized, {width, height});                                                                                      // 缩放
    cv::copyMakeBorder(resized, boxed, py, 480 - height - py, px, 640 - width - px, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));  // 补图

    detector.setInput(cv::dnn::blobFromImage(boxed, 1. / 255., {640, 480}, {}, true));


    return decode_rune_targets(detector.forward(), scale, px, py, image.size());  //  返回targets（二维，屏幕坐标
}

RunePose estimate_rune_pose(const RuneDetection& detection, const cv::Mat& camera, const cv::Mat& distortion, const State& state) {  //  算pnp  输出世界坐标
    RunePose pose;
    if (!detection.found) return pose;

    const double r = kRuneRadiusM, h = kRuneHalfDiagonalM;

    std::vector<cv::Point3d> object = {{0, -r - h, 0}, {-h, -r, 0}, {0, 0, 0}, {h, -r, 0}, {0, -r + h, 0}};
    std::vector<cv::Point2f> pixels(detection.points.begin(), detection.points.end());

    cv::Mat rotation, translation, matrix;

    if (!cv::solvePnP(object, pixels, camera, distortion, rotation, translation, false, cv::SOLVEPNP_IPPE)) return pose;

    cv::Rodrigues(rotation, matrix);
    cv::Point3d center(translation.at<double>(0), translation.at<double>(1), translation.at<double>(2));           // 符心
    cv::Point3d radial(-r * matrix.at<double>(0, 1), -r * matrix.at<double>(1, 1), -r * matrix.at<double>(2, 1));  // 半径向量
    cv::Point3d normal(matrix.at<double>(0, 2), matrix.at<double>(1, 2), matrix.at<double>(2, 2));                 // z轴（旋转轴

    if (!cv::checkRange(matrix) || !cv::checkRange(translation) || center.z <= 0 || center.z + radial.z <= 0) {
        return pose;
    }

    pose.center_m = camera_to_world(center, state);  // 共用普通装甲板的函数    坐标转换成绝对的世界坐标
    pose.radial_m = camera_to_world(center + radial, state) - pose.center_m;
    pose.normal = camera_to_world(normal, state) - state.origin_m;
    pose.valid = true;

    return pose;
}

void RuneMotion::reset() {
    previous_stamp_s = 0;
    speed_rad_s = 0;
    ready = false;
}


void RuneMotion::update(const RunePose& pose, double stamp_s) {  // 跳过前后两帧来算出角速度
    double elapsed = stamp_s - previous_stamp_s;


    if (previous_stamp_s > 0 && elapsed > 0 && elapsed <= kMaximumTrackingGapS) {
        cv::Point3d normal = pose.normal;

        if (normal.dot(previous_normal) < 0)  // 夹角超过180
            normal = -normal;

        double angle = std::atan2(normal.dot(previous_radial.cross(pose.radial_m)), previous_radial.dot(pose.radial_m));

        // 换到另一各板时减去72的整数倍，防止大角度跳跃
        double measured = remainder(angle, 2 * kPi / 5) / elapsed;

        if (isfinite(measured) && abs(measured) <= 5) {
            speed_rad_s = ready ? 0.75 * speed_rad_s + 0.25 * measured : measured;
            ready = true;  // 已有角速度
        } else
            reset();


    } else
        reset();


    previous_radial = pose.radial_m;
    previous_normal = pose.normal;
    previous_stamp_s = stamp_s;
}

cv::Point3d predict_rune_position(const RunePose& pose, double speed, double horizon) {
    double angle = speed * horizon;

    // Rodrigues圆周公式
    const auto& v = pose.radial_m;
    const auto& n = pose.normal;
    return pose.center_m + v * cos(angle) + n.cross(v) * sin(angle) + n * (n.dot(v) * (1 - cos(angle)));
}


void SmallRunePipeline::reset() {
    motion.reset();
    lost_frames = 1;
}

AimCommand SmallRunePipeline::process(cv::Mat& image, const cv::Mat& camera, const cv::Mat& distortion, const State& state) {
    const auto started = std::chrono::steady_clock::now();

    auto targets = detector.detect(image);

    RuneDetection detection;
    if (!targets.empty())
        detection = targets.front();  // 取第一个


    RunePose pose = estimate_rune_pose(detection, camera, distortion, state);

    if (!pose.valid) {
        reset();
        reason = "small_rune_target_lost";
        return {};
    }


    if (motion.previous_stamp_s > 0 && pose.normal.dot(motion.previous_normal) < 0)  // 防止法线方向算反了
        pose.normal = -pose.normal;


    motion.update(pose, state.stamp_s);
    lost_frames = 0;

    double processing = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    double flight = cv::norm(pose.center_m + pose.radial_m - state.origin_m) / state.speed_m_s;  // 飞行时间

    AimCommand command;
    for (int i = 0; i < 3; i++) {  // 三次迭代，修正

        double horizon = flight + max(0.0, state.age_s) + processing + kProcessingDelayS;

        Target target;
        target.found = true;
        target.position_m = predict_rune_position(pose, motion.ready ? motion.speed_rad_s : 0.0, horizon);  // 预测

        command = solve_aim(target, state);  // 和装甲板用的一样的弹道求解

        if (!command.valid) break;

        flight = command.flight_s;
    }
    // for (const auto& point : detection.points)
    //     cv::circle(image, point, 3, cv::Scalar(0, 255, 255), -1);
    // cv::Point2f center = (detection.points[0] + detection.points[1] + detection.points[3] + detection.points[4]) * .25f;
    // cv::line(image, detection.points[2], center, cv::Scalar(255, 0, 255), 1);
    // cv::circle(image, center, 5, cv::Scalar(0, 255, 0), 1);

    reason = motion.ready ? "small_rune_predicting" : "small_rune_first_frame_follow";

    return command;
}
}  // namespace aim
