#include "RL_inference.h"

RL_Inference::RL_Inference(const std::string &policy_path)
{
    try
    {
        policy = torch::jit::load(policy_path);
        policy.eval();
    }
    catch (const c10::Error &e)
    {
        std::cerr << "Error loading the model: " << e.what() << std::endl;
    }

    for (int i = 0; i < FRAME_STACK; i++)
    {
        hist_obs.push_back(std::vector<float>(OBS_DIM, 0.0f));
    }
}

std::vector<float> RL_Inference::infer(double time, std::vector<float> &cmd, std::vector<float> &q, std::vector<float> &dq, std::vector<float> &omega, std::vector<float> &eul_ang)
{
    obs_data[0] = sin(2 * PI * time / PERIOD);
    obs_data[1] = cos(2 * PI * time / PERIOD);
    obs_data[2] = cmd[0] * normalization.lin_vel_scale;
    obs_data[3] = cmd[1] * normalization.lin_vel_scale;
    obs_data[4] = cmd[2] * normalization.ang_vel_scale;

    for (int i = 5; i < 15; i++)
    {
        obs_data[i] = (q[i - 5] - default_angles[i - 5]) * normalization.dof_pos_scale;
    }
    for (int i = 15; i < 25; i++)
    {
        obs_data[i] = dq[i - 15] * normalization.dof_vel_scale;
    }
    for (int i = 25; i < 35; i++)
    {
        obs_data[i] = action[i - 25];
    }
    for (int i = 35; i < 38; i++)
    {
        obs_data[i] = omega[i - 35];
    }
    for (int i = 38; i < 41; i++)
    {
        obs_data[i] = eul_ang[i - 38];
    }

    clip(obs_data, -normalization.clip_obs, normalization.clip_obs);
    hist_obs.push_back(obs_data);
    hist_obs.pop_front();
    for (int i = 0; i < FRAME_STACK; i++)
    {
        for (int j = 0; j < OBS_DIM; j++)
        {
            policy_input[i * OBS_DIM + j] = hist_obs[i][j];
        }
    }

    auto obs_tensor = torch::from_blob(policy_input.data(), {1, (long)policy_input.size()}).clone();
    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(obs_tensor);
    at::Tensor output = policy.forward(inputs).toTensor();
    for (int i = 0; i < ACT_DIM; i++)
        action[i] = output[0][i].item<float>();
    clip(action, -normalization.clip_act, normalization.clip_act);

    for (int i = 0; i < ACT_DIM; i++)
    {
        target_q[i] = action[i] * normalization.action_scale + default_angles[i];
        // tau[i] = kps[i] * (target_q[i] - q[i]) + kds[i] * (target_dq[i] - dq[i]);
    }
    
    // clip(tau, -torque_limit, torque_limit);

    return target_q;
}