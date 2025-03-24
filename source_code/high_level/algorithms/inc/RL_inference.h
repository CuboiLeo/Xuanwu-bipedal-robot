#ifndef RL_INFERENCE_H
#define RL_INFERENCE_H

#include <torch/script.h>
#include <deque>
#include "robot_types.h"
#include "robot.h"
#include "user_math.h"

class RL_Inference
{
public:
    RL_Inference(const std::string &policy_path);
    std::vector<float> infer(double time, std::vector<float> &cmd, std::vector<float> &q, std::vector<float> &dq, std::vector<float> &omega, std::vector<float> &eul_ang);

private:
    static constexpr int OBS_DIM = 41;
    static constexpr int ACT_DIM = 10;
    static constexpr int FRAME_STACK = 15;
    static constexpr double PERIOD = 0.64f;

    torch::jit::script::Module policy;
    std::vector<float> obs_data = std::vector<float>(OBS_DIM, 0.0f);
    std::vector<float> action = std::vector<float>(ACT_DIM, 0.0f);
    std::vector<float> policy_input = std::vector<float>(OBS_DIM*FRAME_STACK, 0.0f);
    std::vector<float> target_q = std::vector<float>(ACT_DIM, 0.0f);
    std::vector<float> target_dq = std::vector<float>(ACT_DIM, 0.0f);
    std::vector<float> tau = std::vector<float>(ACT_DIM, 0.0f);
    std::deque<std::vector<float>> hist_obs;

    struct normalization_t
    {
        float clip_obs = 18.0f;
        float clip_act = 18.0f;
        float lin_vel_scale = 2.0f;
        float ang_vel_scale = 1.0f;
        float dof_pos_scale = 1.0f;
        float dof_vel_scale = 0.05f;
        float action_scale = 0.25f;
    };
    normalization_t normalization;

    std::vector<float> default_angles = {0.0f, 0.0f, 0.152f, -0.36f, 0.208f, 0.0f, 0.0f, 0.152f, -0.36f, 0.208f};
    std::vector<float> kps = {30.0f, 30.0f, 30.0f, 30.0f, 10.0f, 30.0f, 30.0f, 30.0f, 30.0f, 10.0f};
    std::vector<float> kds = std::vector<float>(10, 1.0f);
    float torque_limit = 3.0f;
};

#endif // RL_INFERENCE_H