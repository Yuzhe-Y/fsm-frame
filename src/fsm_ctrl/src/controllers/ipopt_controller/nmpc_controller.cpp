#include "fsm_ctrl/controllers/ipopt_controller/nmpc_controller.hpp"

NmpcController::NmpcController(double ctrl_T, 
                               std::array<double, 2> total_force_limit, 
                               std::array<double, 2> w_limit,
                               int nlp_predict_step, 
                               double nlp_onestep_time, 
                               int nlp_state_num, 
                               int nlp_input_num,
                               Eigen::Matrix<float, 3, 1> nlp_costQ_pos,
                               Eigen::Matrix<float, 3, 1> nlp_costQ_vel,
                               Eigen::Matrix<float, 3, 1> nlp_costQ_quat,
                               Eigen::Matrix<float, 3, 1> nlp_costR_w,
                               double nlp_costR_totalF)
{
    //固定参数赋值，包括控制器周期，输入约束，状态约束，nlp问题求解步数，单步时长，状态变量数目，输入变量数目，代价函数系数
    ctrl_T_ = ctrl_T; //
    total_force_limit_ = total_force_limit; //
    w_limit_ = w_limit; //
    nlp_predict_step_ = nlp_predict_step; //
    nlp_onestep_time_ = nlp_onestep_time; //
    nlp_state_num_ = nlp_state_num; //
    nlp_input_num_ = nlp_input_num; //
    nlp_costQ_pos_ = nlp_costQ_pos;
    nlp_costQ_vel_ = nlp_costQ_vel;
    nlp_costQ_quat_ = nlp_costQ_quat;
    nlp_costR_w_ = nlp_costR_w;
    nlp_costR_totalF_ = nlp_costR_totalF;

    // 初始化初始猜测值
    for (int i = 0; i < nlp_input_num_ * nlp_predict_step_; i++)
    {nlp_initial_u0_.push_back(0);}

    /* states */
    casadi::SX p = casadi::SX::sym("p", 3);             // position in world frame 3*1
    casadi::SX v = casadi::SX::sym("v", 3);             // velocity in world frame 3*1
    casadi::SX q = casadi::SX::sym("q", 4);             // attitude quaternion from world frame to body frame 3*1
    /* inputs */
    casadi::SX w = casadi::SX::sym("w", 3);             // angle velocity in body frame 3*1
    casadi::SX totalF = casadi::SX::sym("totalF", 1);   // motor thrust in body frame 4*1

    /* total states and total inputs */
    casadi::SX x = casadi::SX::vertcat({p, v, q}); // x:10*1
    casadi::SX u = casadi::SX::vertcat({w, totalF}); // u:4*1

    /* dynamics model */
    casadi::SX x_dot = casadi::SX::vertcat({
        v(0),
        v(1),
        v(2),
        -2*(q(0)*q(2) + q(1)*q(3))* totalF,
        -2*(q(2)*q(3) - q(0)*q(1))* totalF,
        world_gravity_acc_ - (q(0)*q(0) - q(1)*q(1) - q(2)*q(2) + q(3)*q(3))* totalF,
        0.5*(-q(1)*w(0) - q(2)*w(1) - q(3)*w(2)),
        0.5*(q(0)*w(0) + q(2)*w(2) - q(3)*w(1)),
        0.5*(q(0)*w(1) - q(1)*w(2) + q(3)*w(0)),
        0.5*(q(0)*w(2) + q(1)*w(1) - q(2)*w(0))
    }); // x_dot:13*1

    //定义模型函数
    casadi::Function model_function = casadi::Function("f", {x, u}, {x_dot});

    //求解问题符号表示
    casadi::SX U = casadi::SX::sym("U", nlp_input_num_, nlp_predict_step_);  //控制输入，本案例中为 4*(predict_step)
    casadi::SX X = casadi::SX::sym("X", nlp_state_num_, nlp_predict_step_ + 1);  //状态输出，本案例中为 13*(predict_step+1)

    //优化问题所需参数 当前所有状态+期望轨迹点对应位置、速度、姿态、加速度+期望控制量 10+(1+predict_step)*10+predict_step*4
    casadi::SX opt_param = casadi::SX::sym("opt_param", (nlp_predict_step_ + 2) * nlp_state_num_ + nlp_predict_step_ * nlp_input_num_);
    // std::cout << "SX 变量 opt_param:" << opt_param << std::endl;

    //优化变量 (predict_step*input_num)*1，从上到下分别为第一个控制量的predict_step步，第二个控制量的predict_step步，...，第input_num个控制量的predict_step步
    casadi::SX opt_var = casadi::SX::reshape(U.T(), -1, 1);
    // std::cout << "SX 变量 opt_var:" << opt_var << std::endl;

    //根据模型函数前向预测无人机运动状态
    X(casadi::Slice(), 0) = opt_param(casadi::Slice(0, nlp_state_num_, 1));  //状态初始值

    for (int i = 0; i < nlp_predict_step_; i++)
    {
        std::vector<casadi::SX> input_X; //最终为10*1
        casadi::SX X_current = X(casadi::Slice(), i);
        casadi::SX U_current = casadi::SX::vertcat({opt_var(i),opt_var(i+nlp_predict_step_),opt_var(i+2*nlp_predict_step_),opt_var(i+3*nlp_predict_step_)});
        // std::cout << "SX 变量 U_current:" << U_current << std::endl;
        input_X.push_back(X_current);
        input_X.push_back(U_current);
        X(casadi::Slice(), i + 1) = model_function(input_X).at(0) * nlp_onestep_time_ + X_current;
    }

    //惩罚矩阵
    casadi::SX costQpos = casadi::SX::diag({casadi::SX::vertcat({ nlp_costQ_pos_(0), nlp_costQ_pos_(1), nlp_costQ_pos_(2)})});
    casadi::SX costQvel = casadi::SX::diag({casadi::SX::vertcat({ nlp_costQ_vel_(0), nlp_costQ_vel_(1), nlp_costQ_vel_(2)})});
    casadi::SX costQquat = casadi::SX::diag({casadi::SX::vertcat({ nlp_costQ_quat_(0), nlp_costQ_quat_(1), nlp_costQ_quat_(2)})});
    casadi::SX costRw = casadi::SX::diag({casadi::SX::vertcat({ nlp_costR_w_(0), nlp_costR_w_(1), nlp_costR_w_(2)})});

    //计算代价函数
    casadi::SX cost_function = casadi::SX::sym("cost_function");
    cost_function = 0;
    for (int i = 0; i < nlp_predict_step_; i++)
    {
        casadi::SX error_pos = opt_param(casadi::Slice(nlp_state_num_*(i+1), nlp_state_num_*(i+1)+3, 1)) - X(casadi::Slice(0,3,1), i);
        casadi::SX error_vel = opt_param(casadi::Slice(nlp_state_num_*(i+1)+3, nlp_state_num_*(i+1)+6, 1)) - X(casadi::Slice(3,6,1), i);
        casadi::SX error_quat = casadi::SX::vertcat({X(6, i)*opt_param(nlp_state_num_*(i+1)+7) - X(7, i)*opt_param(nlp_state_num_*(i+1)+6) + X(8, i)*opt_param(nlp_state_num_*(i+1)+9) - X(9, i)*opt_param(nlp_state_num_*(i+1)+8),
                                                     X(6, i)*opt_param(nlp_state_num_*(i+1)+8) - X(7, i)*opt_param(nlp_state_num_*(i+1)+9) - X(8, i)*opt_param(nlp_state_num_*(i+1)+6) + X(9, i)*opt_param(nlp_state_num_*(i+1)+7),
                                                     X(6, i)*opt_param(nlp_state_num_*(i+1)+9) + X(7, i)*opt_param(nlp_state_num_*(i+1)+8) - X(8, i)*opt_param(nlp_state_num_*(i+1)+7) - X(9, i)*opt_param(nlp_state_num_*(i+1)+6)});
        casadi::SX error_w = opt_param(casadi::Slice((nlp_state_num_*(nlp_predict_step_+2)+nlp_input_num_*i), (nlp_state_num_*(nlp_predict_step_+2)+nlp_input_num*i+3), 1)) - U(casadi::Slice(0,3,1), i);
        casadi::SX error_totalF = opt_param(nlp_state_num_*(nlp_predict_step_+2)+nlp_input_num_*i+3) - U(3, i);
        cost_function = cost_function + casadi::SX::mtimes({ error_pos.T(), costQpos, error_pos }) +
                                        casadi::SX::mtimes({ error_vel.T(), costQvel, error_vel }) + 
                                        casadi::SX::mtimes({ error_quat.T(), costQquat, error_quat }) + 
                                        casadi::SX::mtimes({ error_w.T(), costRw, error_w }) +
                                        nlp_costR_totalF_ * error_totalF * error_totalF;
    }

    casadi::SX error_pos = opt_param(casadi::Slice(nlp_state_num_*(nlp_predict_step_+1), nlp_state_num_*(nlp_predict_step_+1)+3, 1)) - X(casadi::Slice(0,3,1), nlp_predict_step_);
    casadi::SX error_vel = opt_param(casadi::Slice(nlp_state_num_*(nlp_predict_step_+1)+3, nlp_state_num_*(nlp_predict_step_+1)+6, 1)) - X(casadi::Slice(3,6,1), nlp_predict_step_);
    casadi::SX error_quat = casadi::SX::vertcat({X(6, nlp_predict_step_)*opt_param(nlp_state_num_*(nlp_predict_step_+1)+7) - X(7, nlp_predict_step_)*opt_param(nlp_state_num_*(nlp_predict_step_+1)+6) + X(8, nlp_predict_step_)*opt_param(nlp_state_num_*(nlp_predict_step_+1)+9) - X(9, nlp_predict_step_)*opt_param(nlp_state_num_*(nlp_predict_step_+1)+8),
                                                 X(6, nlp_predict_step_)*opt_param(nlp_state_num_*(nlp_predict_step_+1)+8) - X(7, nlp_predict_step_)*opt_param(nlp_state_num_*(nlp_predict_step_+1)+9) - X(8, nlp_predict_step_)*opt_param(nlp_state_num_*(nlp_predict_step_+1)+6) + X(9, nlp_predict_step_)*opt_param(nlp_state_num_*(nlp_predict_step_+1)+7),
                                                 X(6, nlp_predict_step_)*opt_param(nlp_state_num_*(nlp_predict_step_+1)+9) + X(7, nlp_predict_step_)*opt_param(nlp_state_num_*(nlp_predict_step_+1)+8) - X(8, nlp_predict_step_)*opt_param(nlp_state_num_*(nlp_predict_step_+1)+7) - X(9, nlp_predict_step_)*opt_param(nlp_state_num_*(nlp_predict_step_+1)+6)});
    cost_function = cost_function + casadi::SX::mtimes({ error_pos.T(), costQpos, error_pos }) +
                                    casadi::SX::mtimes({ error_vel.T(), costQvel, error_vel }) + 
                                    casadi::SX::mtimes({ error_quat.T(), costQquat, error_quat });   

    //构建求解器（不考虑优化问题）
    //这里的变量说明可以找Casadi C++ API手册
    casadi::SXDict nlp_problem = {
        { "f", cost_function },
        { "x", opt_var },    //系统输出
        { "p", opt_param },  //优化参数,即当前状态、目标状态、目标控制
    };

    std::string solver_name = "ipopt";
    casadi::Dict nlp_opts;
    nlp_opts["expand"] = true;
    nlp_opts["ipopt.max_iter"] = 5000;
    nlp_opts["ipopt.print_level"] = 0;
    nlp_opts["print_time"] = 0;
    nlp_opts["ipopt.acceptable_tol"] = 1e-4;
    nlp_opts["ipopt.acceptable_obj_change_tol"] = 1e-4;
    nlp_opts["ipopt.acceptable_dual_inf_tol"] = 1e-4;

    nlp_solver_ = nlpsol("solver", solver_name, nlp_problem, nlp_opts);
}

NmpcController& NmpcController::operator=(const NmpcController& other)
{
    if (this != &other)
    {
        ctrl_T_ = other.ctrl_T_; //
        total_force_limit_ = other.total_force_limit_; //
        w_limit_ = other.w_limit_; //
        nlp_predict_step_ = other.nlp_predict_step_; //
        nlp_onestep_time_ = other.nlp_onestep_time_; //
        nlp_state_num_ = other.nlp_state_num_; //
        nlp_input_num_ = other.nlp_input_num_; //
        nlp_costQ_pos_ = other.nlp_costQ_pos_; //
        nlp_costQ_vel_ = other.nlp_costQ_vel_; //
        nlp_costQ_quat_ = other.nlp_costQ_quat_; //
        nlp_costR_w_ = other.nlp_costR_w_; //
        nlp_costR_totalF_ = other.nlp_costR_totalF_; //
    }
    return *this;
}

void NmpcController::OptimalSolution(std::vector<double> _current_states,
                                     std::vector<double> _desired_params)
{
    size_t current_states_size = static_cast<size_t>(nlp_state_num_);
    size_t desired_params_size = static_cast<size_t>((nlp_predict_step_ + 1) * nlp_state_num_ + nlp_predict_step_ * nlp_input_num_);
    //向量大小检查
    if (_current_states.size() != current_states_size) 
    {
        std::cerr << "Warning! Current States Vector size changes!" << std::endl;
    }
    if (_desired_params.size() != desired_params_size) 
    {
        std::cerr << "Warning! Desired Params Vector size changes!" << std::endl;
    }

    //控制约束
    std::vector<double> lbx;                   //控制下限
    std::vector<double> ubx;                   //控制上限
    std::vector<double> parameters;            //包括当前状态、目标状态、目标控制

    for(int i = 0; i < nlp_input_num_ - 1; i++)
    {
        for(int j = 0; j < nlp_predict_step_; j++)
        {
            lbx.push_back(w_limit_.at(0));
            ubx.push_back(w_limit_.at(1));
        }
    }
    for (int j = 0; j < nlp_predict_step_; j++)  //力限制
    {
        lbx.push_back(total_force_limit_.at(0));
        ubx.push_back(total_force_limit_.at(1));
    }

    // size_t current_states_size = _current_states.size();
    // size_t desired_params_size = _desired_params.size();
    // std::cout << "current_states_size" << lbx.size() << std::endl;

    for (size_t i = 0; i < current_states_size; i++)  //传入参数
    {
        parameters.push_back(_current_states[i]);
    }
    for (size_t i = 0; i < desired_params_size; i++)
    {
        parameters.push_back(_desired_params[i]);
    }
    // std::cout << parameters << std::endl;

    //求解参数设置
    nlp_args_["lbx"] = lbx;
    nlp_args_["ubx"] = ubx;
    nlp_args_["p"] = parameters;
    nlp_args_["x0"] = nlp_initial_u0_;

    //求解
    nlp_result_ = nlp_solver_(nlp_args_);

    //获取优化变量
    std::vector<double> cmd_control_all(nlp_result_.at("x"));
    optimal_command_ = cmd_control_all;
    std::cout << "cmd_control_all: " << cmd_control_all << std::endl;
    std::vector<double> cmd_control_wx, cmd_control_wy, cmd_control_wz, cmd_control_totalF;
    cmd_control_wx.assign(cmd_control_all.begin(), cmd_control_all.begin() + nlp_predict_step_);
    cmd_control_wy.assign(cmd_control_all.begin() + nlp_predict_step_,
                          cmd_control_all.begin() + 2 * nlp_predict_step_);
    cmd_control_wz.assign(cmd_control_all.begin() + 2 * nlp_predict_step_,
                          cmd_control_all.begin() + 3 * nlp_predict_step_);
    cmd_control_totalF.assign(cmd_control_all.begin() + 3 * nlp_predict_step_,
                              cmd_control_all.begin() + 4 * nlp_predict_step_);

    //存储下一时刻最初优化解猜测
    std::vector<double> initial_guess;

    for (int i = 0; i < nlp_predict_step_; i++)
    {
        initial_guess.push_back(cmd_control_wx.at(i));
    }
    for (int i = 0; i < nlp_predict_step_; i++)
    {
        initial_guess.push_back(cmd_control_wy.at(i));
    }
    for (int i = 0; i < nlp_predict_step_; i++)
    {
        initial_guess.push_back(cmd_control_wz.at(i));
    }
    for (int i = 0; i < nlp_predict_step_; i++)
    {
        initial_guess.push_back(cmd_control_totalF.at(i));
    }

    nlp_initial_u0_ = initial_guess;

    //控制序列的第一组作为当前控制量
    w_command_ << cmd_control_wx.front(), cmd_control_wy.front(), cmd_control_wz.front();
    total_force_command_ = cmd_control_totalF.front();
}
