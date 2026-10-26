#pragma once
#include <algorithm>
#include <array>
#include <cstdint>
#include <string>

struct StrategyStage {
    std::string name;
    std::string action;
    double duration_s = 0;
    double yaw_right_deg = 0;
    double pitch_up_deg = 0;
    int end_enemy_id = 0;
    bool end_on_rune_activated = false;
    std::string end_mode = "any";  // any or all



    // 每收到一帧,判断是否应结束
    bool finished(double elapsed, const std::array<int32_t, 7>& healths, bool rune_activated) const {
        int condition_count = 0;  // 配置了几个结束条件
        int finished_count = 0;   // 其中有几个已经满足


        if (duration_s > 0) {
            condition_count++;
            if (elapsed >= duration_s)
                finished_count++;
        }

        if (end_enemy_id >= 1 && end_enemy_id <= 7) {
            condition_count++;
            if (healths[end_enemy_id - 1] == 0)
                finished_count++;
        }



        if (end_on_rune_activated) {
            condition_count++;
            if (rune_activated) finished_count++;
        }


        if (condition_count == 0) return false;
        if (end_mode == "all")
            return finished_count == condition_count;

        else
            return finished_count > 0;  // any
    }
};

struct RuneStageEvents {
    bool activated = false;       // 增益反馈
    int64_t stage_stamp_ns = 0;   // 该stage起点
    int64_t latest_stamp_ns = 0;  // 最近一次增益激活的时间

    // 进入新阶段时清空标记
    void begin(int64_t frame_stamp_ns) {
        activated = false;
        stage_stamp_ns = std::max(frame_stamp_ns, latest_stamp_ns);
    }

    void observe(int64_t stamp_ns, int64_t now_ns, bool result_activated) {
        if (!result_activated) return;

        if (stamp_ns <= latest_stamp_ns || stamp_ns <= stage_stamp_ns)
            return;

        // 这个激活是有效的
        latest_stamp_ns = stamp_ns;

        activated = true;
    }
};
