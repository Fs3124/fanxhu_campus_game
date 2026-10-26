#ifndef AIM_TOOLS_TYPES_HPP
#define AIM_TOOLS_TYPES_HPP
#include <cstdint>
#include <opencv2/core.hpp>
#include <string>
#include <vector>
namespace aim {
constexpr double kPi = 3.141592653589793;
constexpr double kGravity = 9.81;
constexpr double kSceneUnitsPerMeter = 3.0;  //  规则7.3
constexpr double kFallbackSpeed = 23.5;      // 平均蛋速
constexpr double kMaximumAgeS = 0.2;         // 规则里的是 0.5秒
// 帧龄上限是工程设置；上面规则中的0.5秒指命令超时。
constexpr double kMaximumTrackingGapS = 0.1;  // 运动估计的相邻观测间隔保持不变
constexpr double kMaximumPitchDeg = 60.0;    // 规则7.3  最大俯仰脚
constexpr double kProcessingDelayS = 0.03;   // 运算处理时间估计


struct State {
    double yaw_deg = 0, pitch_deg = 0;

    double stamp_s = 0, age_s = 0;  // age是帧年龄
    std::int64_t stamp_ns = 0;      // 保留官方帧原始纳秒，避免double损失精度
    std::uint64_t sequence = 0;     // 官方frame_sequence

    double speed_m_s = kFallbackSpeed;
    cv::Point3d origin_m;  // 位置转换成 前、右、下
    int health = 0, ammo_remaining = 0;
    bool source_valid = false, measured_speed = false, weakened = false;
};

struct Target {
    bool found = false;
    std::vector<cv::Point2f> corners;
    cv::Point3d position_m;
    cv::Point3d velocity_m_s;
    cv::Point2f center_px;
};

struct AimCommand {
    bool valid = false;
    double yaw_deg = 0, pitch_deg = 0, error_deg = 0;
    double flight_s = 0, elevation_deg = 0;
    cv::Point3d position_m;
};


struct GateResult {
    bool send = false, shoot = false;
    std::string reason;
};


class Armor {

   public:
    AimCommand process(cv::Mat& image, const cv::Mat& camera, const cv::Mat& distortion, const State& state, const std::string& color);
    void reset();
    int lost_frames = 1;

   private:
    cv::Point3d previous_position_m, velocity_m_s;
    cv::Point2f previous_center_px;
    double previous_stamp_s = 0;
    void update_tracking(Target& target, const State& state);
};

Target find_armor(cv::Mat& image, const std::string& color, cv::Point2f previous);
void estimate_position(Target& target, const cv::Mat& camera, const cv::Mat& distortion, const State& state);


cv::Point3d camera_to_world(const cv::Point3d& point, const State& state);

AimCommand solve_aim(const Target& target, const State& state);
GateResult decide_gate(const AimCommand& command, const State& state, int lost_frames, bool enable_fire, double alignment_deg);
}  // namespace aim
#endif
