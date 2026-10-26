#pragma once
#include <array>
#include <opencv2/dnn.hpp>

#include "types.hpp"

namespace aim {
struct RuneDetection {
    bool found = false;
    float confidence = 0;
    std::array<cv::Point2f, 5> points;  // 外侧、左、R心、右、内侧
};


struct RunePose {
    bool valid = false;
    cv::Point3d center_m, radial_m, normal;  // 世界符心、半径向量、单位法线
};


// 符 几何初值
constexpr double kRuneRadiusM = 0.7;
constexpr double kRuneHalfDiagonalM = 0.15;

std::vector<RuneDetection> decode_rune_targets(const cv::Mat& output, float scale, int pad_x, int pad_y, cv::Size image_size);
RunePose estimate_rune_pose(const RuneDetection& detection, const cv::Mat& camera, const cv::Mat& distortion, const State& state);
cv::Point3d predict_rune_position(const RunePose& pose, double speed_rad_s, double horizon_s);

struct RuneMotion {
    double speed_rad_s = 0;
    double previous_stamp_s = 0;
    bool ready = false;
    cv::Point3d previous_radial, previous_normal;

    void update(const RunePose& pose, double stamp_s);

    void reset();
};


struct RuneShotCooldown {
    double seconds = 0.5;  // 冷却间隔 （到时候会被yaml中的值覆盖
    int previous_ammo = -1;
    double next_allowed_s = 0;

    bool allow(bool requested, int ammo, double now_s);
};

//   两种符共用一个检测器
class RuneDetector {
   public:
    explicit RuneDetector(const std::string& model_path);
    std::vector<RuneDetection> detect(const cv::Mat& image);

   private:
    cv::dnn::Net detector;
};

class SmallRunePipeline {
   public:
    explicit SmallRunePipeline(RuneDetector& shared_detector) : detector(shared_detector) {}
    AimCommand process(cv::Mat& image, const cv::Mat& camera, const cv::Mat& distortion, const State& state);
    void reset();
    int lost_frames = 1;
    RuneMotion motion;
    std::string reason = "waiting_for_small_rune";

   private:
    RuneDetector& detector;
};
}  // namespace aim
