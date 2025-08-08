#include "trajectory_generator.hpp"
#include <cmath>

namespace trajectory_generator
{
    std::vector<TrajectoryPoint> TrajectoryGenerator::generateCubicTrajectory(
        double start_pos,
        double end_pos,
        double duration,
        double dt)
    {
        std::vector<TrajectoryPoint> trajectory;
        if (duration <= 0 || dt <= 0)
        {
            return trajectory;
        }

        // 三次多项式系数
        double a0 = start_pos;
        double a1 = 0.0; // 初始速度设为0
        double a2 = (3.0 * (end_pos - start_pos)) / (duration * duration);
        double a3 = (-2.0 * (end_pos - start_pos)) / (duration * duration * duration);

        // 生成轨迹点
        for (double t = 0; t <= duration; t += dt)
        {
            TrajectoryPoint point;
            point.time = t;
            point.position = a0 + a1 * t + a2 * t * t + a3 * t * t * t;
            point.velocity = a1 + 2 * a2 * t + 3 * a3 * t * t;
            point.acceleration = 2 * a2 + 6 * a3 * t;
            trajectory.push_back(point);
        }

        // 确保最后一个点精确
        TrajectoryPoint final_point;
        final_point.time = duration;
        final_point.position = end_pos;
        final_point.velocity = 0.0; // 结束速度设为0
        final_point.acceleration = 0.0;
        trajectory.push_back(final_point);

        return trajectory;
    }
}
