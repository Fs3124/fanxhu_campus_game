#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tdt_interface/msg/auto_aim_frame.hpp>
#include <tdt_interface/msg/rune_result.hpp>
#include <tdt_interface/msg/send_data.hpp>

#include "big_rune.hpp"
#include "small_rune.hpp"
#include "strategy.hpp"
#include "types.hpp"

using namespace std;


// 把角度限制在-180,180之间
double wrap_yaw(double angle) {
    if (!std::isfinite(angle)) return angle;
    while (angle >= 180.0) angle -= 360.0;
    while (angle < -180.0) angle += 360.0;
    return angle;
}

class AimNode {
   public:
    rclcpp::Node::SharedPtr node;
    AimNode() {
        cv::setNumThreads(1);  // 单线程

        node = rclcpp::Node::make_shared("aim_node");

        rcl_interfaces::msg::ParameterDescriptor fixed;
        fixed.read_only = true;                                       // 策略只读
        int player = node->declare_parameter("player_id", 1, fixed);  // 读前提参数
        color = node->declare_parameter<std::string>("target_color", "red", fixed);
        enable_fire = node->declare_parameter("enable_fire", false, fixed);
        show_image = node->declare_parameter("show_image", false, fixed);
        print_details = node->declare_parameter("print_details", false, fixed);
        alignment_deg = node->declare_parameter("fire_alignment_deg", 1.0, fixed);

        configuration_valid = (player == 1 || player == 2) && (color == "red" || color == "blue");

        auto order = node->declare_parameter<std::vector<std::string>>("strategy_order", {"opening_turn", "armor"}, fixed);

        bool needs_rune = false;

        for (const auto& name : order) {

            StrategyStage stage;


            stage.name = name;
            std::string prefix = "strategy." + name + ".";
            // 读具体策略战术
            stage.action = node->declare_parameter<std::string>(prefix + "action", "", fixed);
            stage.duration_s = node->declare_parameter(prefix + "duration_s", name == "opening_turn" ? 2.0 : 0.0, fixed);
            stage.yaw_right_deg = node->declare_parameter(prefix + "yaw_right_deg", name == "opening_turn" ? 90.0 : 0.0, fixed);
            stage.pitch_up_deg = node->declare_parameter(prefix + "pitch_up_deg", 0.0, fixed);
            stage.end_enemy_id = node->declare_parameter(prefix + "end_when.enemy_dead", 0, fixed);
            stage.end_on_rune_activated = node->declare_parameter(prefix + "end_when.rune_activated", false, fixed);
            stage.end_mode = node->declare_parameter<std::string>(prefix + "end_when.mode", "any", fixed);


            needs_rune = needs_rune || stage.action == "small_rune" || stage.action == "big_rune";  // 是否能量机关
            stages.push_back(stage);
        }


        if (needs_rune) {
            rune_cooldown.seconds = node->declare_parameter("small_rune_shot_cooldown_s", 0.5, fixed);
            big_rune_cooldown.seconds = node->declare_parameter("big_rune_shot_cooldown_s", 0.5, fixed);


            cv::setNumThreads(4);  // 大小符共用，固定4线程。
            std::string model = ament_index_cpp::get_package_share_directory("aim_tools") + "/models/model-0624.onnx";
            rune_detector = std::make_unique<aim::RuneDetector>(model);  // 大小符共用这一个检测器

            if (std::any_of(stages.begin(), stages.end(), [](const auto& s) { return s.action == "small_rune"; }))  //  看一下策略里面有没有能量机关相关的(any_of 等价于用for遍历一个一个找)
            {
                small_rune = std::make_unique<aim::SmallRunePipeline>(*rune_detector);
            }
            if (std::any_of(stages.begin(), stages.end(), [](const auto& s) { return s.action == "big_rune"; })) {
                big_rune = std::make_unique<aim::BigRunePipeline>(*rune_detector);
            }
        }


        publisher = node->create_publisher<tdt_interface::msg::SendData>("/target_angles_player_" + std::to_string(player), 1);
        subscription = node->create_subscription<tdt_interface::msg::AutoAimFrame>("/autoaim/frame", rclcpp::QoS(1).reliable(), [this](tdt_interface::msg::AutoAimFrame::ConstSharedPtr frame) { receive(*frame); });

        rune_subscription = node->create_subscription<tdt_interface::msg::RuneResult>("/game/rune/result", rclcpp::QoS(10).reliable(), [this](tdt_interface::msg::RuneResult::ConstSharedPtr result) {
            rune_events.observe(rclcpp::Time(result->header.stamp).nanoseconds(), node->now().nanoseconds(), result->activated);
        });  // 订阅rune相关


        timer = node->create_wall_timer(std::chrono::milliseconds(50), [this]() { check_silence(); });  // 检查是否断流  （20hz


        RCLCPP_INFO(node->get_logger(), "startup player=%d fire=%s", player, enable_fire ? "true" : "false");
    }

   private:
    aim::Armor armor;

    std::unique_ptr<aim::RuneDetector> rune_detector;  // 让ONNX 文件加载到内存  方便后面每帧调用
    std::unique_ptr<aim::SmallRunePipeline> small_rune;
    std::unique_ptr<aim::BigRunePipeline> big_rune;

    aim::RuneShotCooldown rune_cooldown;  // 用于强制冷却的对象 （小符专用
    aim::RuneShotCooldown big_rune_cooldown;
    RuneStageEvents rune_events;

    std::vector<StrategyStage> stages;
    std::size_t stage_index = 0;
    bool stage_started = false;

    double stage_started_s = 0;
    double turn_yaw_deg = 0;
    double turn_pitch_deg = 0;

    aim::State state;
    std::string color, previous_reason;
    bool enable_fire, show_image, print_details, configuration_valid;
    double alignment_deg, last_stamp_s = 0;
    uint64_t last_sequence = 0;

    rclcpp::Publisher<tdt_interface::msg::SendData>::SharedPtr publisher;
    rclcpp::Subscription<tdt_interface::msg::AutoAimFrame>::SharedPtr subscription;
    rclcpp::Subscription<tdt_interface::msg::RuneResult>::SharedPtr rune_subscription;
    rclcpp::TimerBase::SharedPtr timer;


    void read_state(const tdt_interface::msg::AutoAimFrame& frame) {  // 读取此帧玩家状态以及符的增益激活情况   同时判断有没有非法数据

        state.speed_m_s = aim::kFallbackSpeed;
        state.stamp_s = rclcpp::Time(frame.header.stamp).seconds();
        state.stamp_ns = rclcpp::Time(frame.header.stamp).nanoseconds();
        state.sequence = frame.state.frame_sequence;
        state.age_s = node->now().seconds() - state.stamp_s;

        state.yaw_deg = frame.state.yaw_degrees;
        state.pitch_deg = frame.state.pitch_degrees;

        state.health = frame.state.health;
        state.ammo_remaining = frame.state.ammo_remaining;
        state.weakened = frame.state.is_weakened;
        state.origin_m = {frame.state.position.x / 3, -frame.state.position.y / 3, -frame.state.position.z / 3};

        state.measured_speed = frame.state.projectile_speed / 3 >= 23.2 && frame.state.projectile_speed / 3 <= 23.8;
        // 检查蛋速范围
        if (state.measured_speed) state.speed_m_s = frame.state.projectile_speed / 3;

        state.source_valid = configuration_valid && state.stamp_s > last_stamp_s && frame.state.frame_sequence > last_sequence &&
                             frame.header.stamp == frame.state.header.stamp && frame.header.stamp == frame.camera_info.header.stamp &&
                             std::isfinite(state.yaw_deg) && state.yaw_deg >= -180 && state.yaw_deg < 180 &&
                             std::isfinite(state.pitch_deg) && std::abs(state.pitch_deg) <= aim::kMaximumPitchDeg &&
                             std::isfinite(state.origin_m.x) && std::isfinite(state.origin_m.y) && std::isfinite(state.origin_m.z);


        last_stamp_s = max(last_stamp_s, state.stamp_s);
        last_sequence = max(last_sequence, frame.state.frame_sequence);  // 防止被旧帧弄乱
    }


    //            图像输入  检查
    cv::Mat decode(const tdt_interface::msg::AutoAimFrame& frame) {  // 转换成opencv可读类型

        if (!state.source_valid || state.age_s < 0 || state.age_s > aim::kMaximumAgeS) return cv::Mat();
        if (!frame.compressed_image.data.empty() && frame.image.data.empty() &&
            frame.compressed_image.header.stamp == frame.header.stamp &&
            frame.compressed_image.format.find("jpeg") != std::string::npos)
            return cv::imdecode(frame.compressed_image.data, cv::IMREAD_COLOR);

        return cv::Mat();
    }

    void receive(const tdt_interface::msg::AutoAimFrame& frame) {  //    /autoaim/frame的订阅回调函数

        read_state(frame);
        cv::Mat image = decode(frame);
        cv::Mat camera(3, 3, CV_64F);  //  camera =K
        bool camera_valid = true;

        for (int index = 0; index < 9; ++index) {  // 组装相机内参

            camera.at<double>(index / 3, index % 3) = frame.camera_info.k[index];
            if (!std::isfinite(frame.camera_info.k[index])) camera_valid = false;
        }
        camera_valid = camera_valid && camera.at<double>(0, 0) > 0 && camera.at<double>(1, 1) > 0 && camera.at<double>(2, 2) == 1;


        for (double i : frame.camera_info.d)  // 所有畸变系数都必须是  0
            if (i != 0)
                camera_valid = false;



        state.age_s = node->now().seconds() - state.stamp_s;
        bool usable = state.source_valid && state.age_s >= 0 && state.age_s <= aim::kMaximumAgeS && state.health > 0;

        if (!usable || image.empty() || !camera_valid) {
            reset_tracking();
            publish({}, {false, false, "stale_or_invalid_source"});
            return;
        }


        // 执行策略
        if (stage_index == stages.size()) {
            publish({}, {false, false, "strategy finished"});  // 防止空策略或者已完全结束策略
            return;
        }
        double steady_s = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();  // 获取某一阶段开始时的时间点

        if (stage_started && stages[stage_index].finished(steady_s - stage_started_s, frame.state.enemy_healths, rune_events.activated)) {  // 该阶段开始过了并且经finished判断已经结束了
            RCLCPP_INFO(node->get_logger(), "stage_finished=%s", stages[stage_index].name.c_str());

            stage_index++;
            stage_started = false;
            reset_tracking();
            publish({}, {false, false, "stage_transition"});  //  下一帧执行新阶段。
            return;
        }


        if (stage_index == stages.size()) {
            publish({}, {false, false, "strategy finished"});  // 防止恰好完全结束策略
            return;
        }


        //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        const auto& stage = stages[stage_index];

        if (!stage_started) {
            stage_started = true;
            stage_started_s = steady_s;

            rune_events.begin(state.stamp_ns);  // 新阶段来了，清空符的事件状态

            turn_yaw_deg = wrap_yaw(state.yaw_deg + stage.yaw_right_deg);
            turn_pitch_deg = state.pitch_deg - stage.pitch_up_deg;  // pitch向下为正

            RCLCPP_INFO(node->get_logger(), "stage=%s action=%s", stage.name.c_str(), stage.action.c_str());
        }


        if (stage.action == "turn" || stage.action == "wait") {
            // 转头期间发固定角度
            aim::AimCommand view;
            view.yaw_deg = stage.action == "turn" ? turn_yaw_deg : state.yaw_deg;  // 不是turn的话就还是之前的度数
            view.pitch_deg = stage.action == "turn" ? turn_pitch_deg : state.pitch_deg;
            if (abs(view.pitch_deg) > aim::kMaximumPitchDeg) {
                publish({}, {false, false, "strategy_pitch_out_of_range"});
                return;
            }
            publish(view, {true, false, stage.action});
            return;
        }

        aim::AimCommand command;
        bool prediction_ready = true;
        int lost_frames = 1;
        const auto processing_started = std::chrono::steady_clock::now();

        if (stage.action == "small_rune") {
            command = small_rune->process(image, camera, cv::Mat(), state);  // 畸变为空
            lost_frames = small_rune->lost_frames;
            prediction_ready = small_rune->motion.ready;
        } else if (stage.action == "big_rune") {
            command = big_rune->process(image, camera, cv::Mat(), state);
            lost_frames = big_rune->lost_frames;
            prediction_ready = big_rune->prediction_ready;
        } else {
            command = armor.process(image, camera, cv::Mat(), state, color);
            lost_frames = armor.lost_frames;
        }


        //   输出能量机关相关调试信息
        state.age_s = node->now().seconds() - state.stamp_s;  // 包括本帧模型推理的耗时
        if (stage.action == "small_rune") {
            double processing_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - processing_started).count();
            RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                                 "small_rune reason=%s processing_ms=%.1f frame_age_ms=%.1f prediction_ready=%s omega_rad_s=%.3f command_valid=%s",
                                 small_rune->reason.c_str(), processing_ms, state.age_s * 1000,
                                 prediction_ready ? "true" : "false", small_rune->motion.speed_rad_s, command.valid ? "true" : "false");
        }


        if (stage.action == "big_rune") {
            double processing_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - processing_started).count();
            RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
                                 "big_rune reason=%s unhit=%d samples=%zu processing_ms=%.1f frame_age_ms=%.1f prediction_ready=%s omega=%.3f horizon_ms=%.1f",
                                 big_rune->reason.c_str(), big_rune->unhit_targets, big_rune->fit_samples,
                                 processing_ms, state.age_s * 1000, prediction_ready ? "true" : "false",
                                 big_rune->speed_rad_s, big_rune->prediction_horizon_s * 1000);
        }

        aim::GateResult gate = aim::decide_gate(command, state, lost_frames, enable_fire && prediction_ready, alignment_deg);


        //   小符的射击冷却
        if (stage.action == "small_rune") {
            double now_s = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
            bool allowed = rune_cooldown.allow(gate.shoot, state.ammo_remaining, now_s);  // 冷却完毕的时间点存在cooldown这个对象里

            if (gate.shoot && !allowed) gate.reason = "small_rune_cooldown";
            gate.shoot = allowed;  // 只停火，依旧让gate send角度，保持跟枪
        }



        if (stage.action == "big_rune") {
            double now_s = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
            bool allowed = big_rune_cooldown.allow(gate.shoot, state.ammo_remaining, now_s);
            if (gate.shoot && !allowed) gate.reason = "big_rune_cooldown";
            gate.shoot = allowed;
        }

        publish(command, gate);


        // 大符换板的短空档只停火，保留拟合；长失靶由大符内部处理。
        if (!gate.send && !(stage.action == "big_rune" && gate.reason == "target_lost")) reset_tracking();
        if (show_image && !image.empty())
            display(image, command, gate);
    }


    void publish(const aim::AimCommand& command, const aim::GateResult& gate) {
        tdt_interface::msg::SendData message;

        message.yaw = state.yaw_deg;
        message.pitch = state.pitch_deg;
        message.if_shoot = false;

        if (gate.send) {
            message.yaw = command.yaw_deg;
            message.pitch = command.pitch_deg;
            message.if_shoot = gate.shoot;
        }

        message.yaw = static_cast<float>(wrap_yaw(message.yaw));  // jiaodu写成[-180,180)

        if (std::isfinite(message.yaw) && std::isfinite(message.pitch))
            publisher->publish(message);

        if (gate.reason != previous_reason || print_details)  // 只在决策原因变化时或者 print_detail是true 时打印调试信息

            RCLCPP_INFO(node->get_logger(), "aim reason=%s xyz_m=(%.2f,%.2f,%.2f) flight_s=%.3f lift_deg=%.2f yaw=%.2f pitch=%.2f shoot=%s error_deg=%.3f",
                        gate.reason.c_str(), command.position_m.x, command.position_m.y, command.position_m.z,
                        command.flight_s, command.elevation_deg, message.yaw, message.pitch, message.if_shoot ? "true" : "false", command.error_deg);
        previous_reason = gate.reason;
    }

    void reset_tracking() {
        armor.reset();
        if (small_rune)
            small_rune->reset();
        if (big_rune)
            big_rune->reset();
    }

    void check_silence() {  // timer的回调  检查是否断流
        if (last_stamp_s == 0 || previous_reason == "stale_or_invalid_source")
            return;

        state.age_s = node->now().seconds() - state.stamp_s;

        if (state.age_s <= aim::kMaximumAgeS && state.age_s >= 0)
            return;
        reset_tracking();
        publish(aim::AimCommand(), aim::decide_gate(aim::AimCommand(), state, 1, enable_fire, alignment_deg));
    }


    void display(cv::Mat& image, const aim::AimCommand& command, const aim::GateResult& gate) {  // 在小窗口显示一些信息
        std::string text = "yaw=" + std::to_string(command.yaw_deg) + " pitch=" + std::to_string(command.pitch_deg) + " " + gate.reason;
        cv::putText(image, text, cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, .45, cv::Scalar(0, 255, 255), 1);
        cv::imshow("Aiming", image);
        cv::waitKey(1);
    }
};
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    AimNode application;
    rclcpp::spin(application.node);
    rclcpp::shutdown();
    return 0;
}
