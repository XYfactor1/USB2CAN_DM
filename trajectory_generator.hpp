#pragma once
#include <vector>

namespace trajectory_generator
{

    struct TrajectoryPoint
    {
        double position; // rad
        double velocity; // rad/s
        double acceleration;
        double time;
    };

    class TrajectoryGenerator
    {
    public:
        // 生成一个三次多项式轨迹（起点 -> 终点）
        static std::vector<TrajectoryPoint> generateCubicTrajectory(
            double start_pos, // 初始位置(rad)
            double end_pos,   // 目标位置(rad)
            double duration,  // 轨迹总时间(s)
            double dt         // 控制周期(s)，如 0.001 = 1ms
        );
    };
}