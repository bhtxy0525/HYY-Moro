/**
 * @file fdcc_hri_controller.cpp
 * @brief 17-DOF upper-body FDCC framework + Experiment 1-1
 *
 * Structure:
 *   1. Common math/model/hardware helpers: normally do not modify after verification.
 *   2. FDCCS + initFDCC() + fdccControlStep(): common FDCC controller layer.
 *   3. runExperiment1_1(): experiment-specific layer. Future experiments should mainly add/modify runExperimentX().
 *
 * PSG compatibility:
 *   FDCCStepInput contains T_world_base. Experiment 1-1 keeps it Identity.
 *   Future PSG experiments can update T_world_base every cycle while keeping the FDCC core unchanged.
 *
 * HYY configuration:
 *   group 0 -> left arm  (7 DOF)
 *   group 1 -> right arm (7 DOF)
 *   group 2 -> torso     (3 DOF)
 *   F/T sensors -> "left_sensor", "right_sensor"
 */

#include "fdcc_hri_controller.h"

#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/mjcf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/spatial/force.hpp>
#include <pinocchio/spatial/inertia.hpp>
#include <pinocchio/spatial/explog.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "HYYRobotInterface.h"

using namespace HYYRobotBase;

namespace fdcc
{

    const std::string FDCC_MJCF_PATH = "/home/robot/Downloads/wheeled_humanoid_robot/robot.xml";
    const std::string LEFT_EE_FRAME_NAME = "ee_left";
    const std::string RIGHT_EE_FRAME_NAME = "ee_right";

    int CustomMoveA(robjoint *rjoint0, speed *rspeed0, robjoint *rjoint1, speed *rspeed1, robjoint *rjoint2, speed *rspeed2)
    {
        setMoveThread(1);
        MultiMoveA(rjoint0, rspeed0, NULL3, 0);
        MultiMoveA(rjoint1, rspeed1, NULL3, 1);
        MultiMoveA(rjoint2, rspeed2, NULL3, 2);
        MoveSyncStart();
        wait_move_finish(0);
        wait_move_finish(1);
        wait_move_finish(2);
        setMoveThread(0);
        return 0;
    }

    // ============================================================================
    // 1. Common helper functions -- only keep helpers with independent logic
    // ============================================================================
    Vector3d projectParallel(const Vector3d &a, const Vector3d &b)
    {
        const double denom = b.squaredNorm();
        if (denom <= EPS)
            return Vector3d::Zero();
        return (a.dot(b) / denom) * b;
    }

    int initFDCCModel(pinocchio::Model &model, ModelIndex &idx)
    {
        try
        {
            pinocchio::mjcf::buildModel(FDCC_MJCF_PATH, model);
        }
        catch (const std::exception &e)
        {
            std::cerr << "buildModel failed: " << e.what() << std::endl;
            return -1;
        }

        const std::vector<std::string> torso_names = {"J-T1", "J-T2", "J-T3"};
        const std::vector<std::string> left_names = {"J-L1", "J-L2", "J-L3", "J-L4", "J-L5", "J-L6", "J-L7"};
        const std::vector<std::string> right_names = {"J-R1", "J-R2", "J-R3", "J-R4", "J-R5", "J-R6", "J-R7"};

        // 根据关节名称直接建立 Pinocchio 关节索引，不再单独封装简单查询函数。
        idx.torso_joint_ids.clear();
        idx.left_arm_joint_ids.clear();
        idx.right_arm_joint_ids.clear();

        for (const auto &name : torso_names)
        {
            const auto jid = model.getJointId(name);
            if (jid >= model.njoints)
            {
                std::cerr << "torso joint does not exist: " << name << std::endl;
                return -2;
            }
            idx.torso_joint_ids.push_back(jid);
        }

        for (const auto &name : left_names)
        {
            const auto jid = model.getJointId(name);
            if (jid >= model.njoints)
            {
                std::cerr << "left arm joint does not exist: " << name << std::endl;
                return -2;
            }
            idx.left_arm_joint_ids.push_back(jid);
        }

        for (const auto &name : right_names)
        {
            const auto jid = model.getJointId(name);
            if (jid >= model.njoints)
            {
                std::cerr << "right arm joint does not exist: " << name << std::endl;
                return -2;
            }
            idx.right_arm_joint_ids.push_back(jid);
        }

        idx.active_joint_ids.clear();
        idx.active_joint_ids.insert(idx.active_joint_ids.end(), idx.torso_joint_ids.begin(), idx.torso_joint_ids.end());
        idx.active_joint_ids.insert(idx.active_joint_ids.end(), idx.left_arm_joint_ids.begin(), idx.left_arm_joint_ids.end());
        idx.active_joint_ids.insert(idx.active_joint_ids.end(), idx.right_arm_joint_ids.begin(), idx.right_arm_joint_ids.end());
        if (idx.active_joint_ids.size() != 17)
        {
            std::cerr << "Expected 17 active joints, got " << idx.active_joint_ids.size() << std::endl;
            return -3;
        }

        for (int i = 0; i < 17; ++i)
        {
            const auto jid = idx.active_joint_ids[i];
            idx.active_q_idx[i] = static_cast<int>(model.idx_qs[jid]);
            idx.active_v_idx[i] = static_cast<int>(model.idx_vs[jid]);
        }

        idx.left_ee_frame_id = model.getFrameId(LEFT_EE_FRAME_NAME);
        idx.right_ee_frame_id = model.getFrameId(RIGHT_EE_FRAME_NAME);
        if (idx.left_ee_frame_id >= model.nframes || idx.right_ee_frame_id >= model.nframes)
        {
            std::cerr << "Cannot find ee_left / ee_right frame." << std::endl;
            return -4;
        }

        for (auto jid : idx.torso_joint_ids)
            model.inertias[jid] = pinocchio::Inertia(0.5, Vector3d::Zero(), (Vector3d(0.1, 0.1, 0.1)).asDiagonal());
        for (size_t i = 0; i + 1 < idx.left_arm_joint_ids.size(); ++i)
            model.inertias[idx.left_arm_joint_ids[i]] = pinocchio::Inertia(0.005, Vector3d::Zero(), 1e-6 * Eigen::Matrix3d::Identity());
        for (size_t i = 0; i + 1 < idx.right_arm_joint_ids.size(); ++i)
            model.inertias[idx.right_arm_joint_ids[i]] = pinocchio::Inertia(0.005, Vector3d::Zero(), 1e-6 * Eigen::Matrix3d::Identity());
        model.inertias[idx.left_arm_joint_ids.back()] = pinocchio::Inertia(1.0, Vector3d(0.0, 0.0, 0.05), Eigen::Matrix3d::Identity());
        model.inertias[idx.right_arm_joint_ids.back()] = pinocchio::Inertia(1.0, Vector3d(0.0, 0.0, 0.05), Eigen::Matrix3d::Identity());
        model.gravity.linear().setZero();

        std::cout << "FDCC Pinocchio model loaded. nq = " << model.nq << ", nv = " << model.nv << ", njoints = " << model.njoints << std::endl;
        return 0;
    }

    pinocchio::SE3 fkineRelativeBase(const pinocchio::Model &model, pinocchio::Data &data, const Eigen::VectorXd &q, pinocchio::FrameIndex ee_frame_id)
    {
        pinocchio::forwardKinematics(model, data, q);
        pinocchio::updateFramePlacements(model, data);
        return data.oMf[ee_frame_id];
    }

    Eigen::MatrixXd jacobWorldAlignedFull(const pinocchio::Model &model, pinocchio::Data &data, const Eigen::VectorXd &q, pinocchio::FrameIndex ee_frame_id)
    {
        pinocchio::forwardKinematics(model, data, q);
        pinocchio::updateFramePlacements(model, data);
        pinocchio::computeJointJacobians(model, data, q);
        Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, model.nv);
        pinocchio::getFrameJacobian(model, data, ee_frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J);
        return J;
    }

    Vector6d poseError6DWorld(const pinocchio::SE3 &Td, const pinocchio::SE3 &T)
    {
        Vector6d e;
        e.head<3>() = Td.translation() - T.translation();
        e.tail<3>() = pinocchio::log3(Td.rotation() * T.rotation().transpose());
        return e;
    }

    Vector6d transformWrenchDualAdjoint(const pinocchio::SE3 &T_A_B, const Vector6d &F_B)
    {
        Vector6d F_A;
        const Vector3d f_A = T_A_B.rotation() * F_B.head<3>();
        F_A.head<3>() = f_A;
        F_A.tail<3>() = T_A_B.translation().cross(f_A) + T_A_B.rotation() * F_B.tail<3>();
        return F_A;
    }

    Vector6d sensorWrenchLocalToWorldAligned(const pinocchio::SE3 &T_world_ee, const pinocchio::SE3 &T_ee_ft, const Vector6d &F_sensor_local)
    {
        const Vector6d F_ee_local = transformWrenchDualAdjoint(T_ee_ft, F_sensor_local);
        Vector6d F_world;
        F_world.head<3>() = T_world_ee.rotation() * F_ee_local.head<3>();
        F_world.tail<3>() = T_world_ee.rotation() * F_ee_local.tail<3>();
        return F_world;
    }

    Vector6d wrenchWorldAlignedToLocal(const pinocchio::SE3 &T_world_ee, const Vector6d &F_world)
    {
        Vector6d F_local;
        F_local.head<3>() = T_world_ee.rotation().transpose() * F_world.head<3>();
        F_local.tail<3>() = T_world_ee.rotation().transpose() * F_world.tail<3>();
        return F_local;
    }

    Vector6d computeImpedanceWrenchWorld(const Vector6d &xerr, const Vector6d &dx, const Matrix6d &K, const Matrix6d &D, const Vector6d &F_des)
    {
        const Vector6d F_sp = K * xerr;
        Vector6d F_sp_parallel = Vector6d::Zero();
        F_sp_parallel.head<3>() = projectParallel(F_sp.head<3>(), F_des.head<3>());
        F_sp_parallel.tail<3>() = projectParallel(F_sp.tail<3>(), F_des.tail<3>());
        return F_sp - F_sp_parallel - D * dx;
    }

    Vector6d computeControlWrenchPD(const Vector6d &F_net, const Vector6d &F_net_prev, Vector6d &dF_filt, double dt, const Matrix6d &K_F, const Matrix6d &D_F, double alpha)
    {
        const Vector6d dF = (F_net - F_net_prev) / dt;
        dF_filt = (1.0 - alpha) * dF_filt + alpha * dF;
        return K_F * F_net + D_F * dF_filt;
    }

    Eigen::VectorXd forwardDynamicsDual(const pinocchio::Model &model, pinocchio::Data &data, const Eigen::VectorXd &q, const Eigen::VectorXd &dq, const Vector6d &F_left_ee_local, const Vector6d &F_right_ee_local, pinocchio::FrameIndex left_frame_id, pinocchio::FrameIndex right_frame_id)
    {
        const auto left_parent_joint = model.frames[left_frame_id].parentJoint;
        const auto right_parent_joint = model.frames[right_frame_id].parentJoint;
        const pinocchio::SE3 &jM_left_ee = model.frames[left_frame_id].placement;
        const pinocchio::SE3 &jM_right_ee = model.frames[right_frame_id].placement;
        pinocchio::container::aligned_vector<pinocchio::Force> fext(model.njoints, pinocchio::Force::Zero());
        fext[left_parent_joint] = jM_left_ee.act(pinocchio::Force(F_left_ee_local.head<3>(), F_left_ee_local.tail<3>()));
        fext[right_parent_joint] = jM_right_ee.act(pinocchio::Force(F_right_ee_local.head<3>(), F_right_ee_local.tail<3>()));
        return pinocchio::aba(model, data, q, dq, Eigen::VectorXd::Zero(model.nv), fext);
    }

    Vector17d computeJointLimitAcceleration(const Vector17d &q_active, const Vector17d &q_min, const Vector17d &q_max, const std::array<bool, 17> &valid, const Vector17d &margin, double a_lim)
    {
        Vector17d ddq_lim = Vector17d::Zero();
        for (int i = 0; i < 17; ++i)
        {
            if (!valid[i] || margin[i] <= EPS)
                continue;
            const double q_low_safe = q_min[i] + margin[i];
            const double q_high_safe = q_max[i] - margin[i];
            if (q_active[i] < q_low_safe)
            {
                // 将归一化越界量限制到 [0, 1]，并使用 smootherstep 平滑增加关节限位恢复加速度。
                const double s = std::clamp((q_low_safe - q_active[i]) / margin[i], 0.0, 1.0);
                ddq_lim[i] = a_lim * s * s * s * (10.0 - 15.0 * s + 6.0 * s * s);
            }
            else if (q_active[i] > q_high_safe)
            {
                // 上限附近采用相同平滑函数，并施加反方向恢复加速度。
                const double s = std::clamp((q_active[i] - q_high_safe) / margin[i], 0.0, 1.0);
                ddq_lim[i] = -a_lim * s * s * s * (10.0 - 15.0 * s + 6.0 * s * s);
            }
        }
        return ddq_lim;
    }

    // ============================================================================
    // 实验数据记录
    // ============================================================================
    void saveLogCSV(const std::string &path, const std::vector<FDCCLogRow> &log)
    {
        std::ofstream ofs(path);
        if (!ofs.is_open())
        {
            std::cerr << "Cannot open log file: " << path << std::endl;
            return;
        }

        ofs << "time,left_Fx,left_Fy,left_Fz,left_Mx,left_My,left_Mz,"
               "right_Fx,right_Fy,right_Fz,right_Mx,right_My,right_Mz,"
               "left_dx,left_dy,left_dz,right_dx,right_dy,right_dz,"
               "left_position_error,right_position_error,left_rotation_error,right_rotation_error\n";

        for (const auto &row : log)
        {
            ofs << row.t;

            for (int i = 0; i < 6; ++i)
                ofs << ',' << row.F_left_sensor_local[i];

            for (int i = 0; i < 6; ++i)
                ofs << ',' << row.F_right_sensor_local[i];

            for (int i = 0; i < 3; ++i)
                ofs << ',' << row.left_disp_local[i];

            for (int i = 0; i < 3; ++i)
                ofs << ',' << row.right_disp_local[i];

            ofs << ',' << row.left_pos_error
                << ',' << row.right_pos_error
                << ',' << row.left_rot_error
                << ',' << row.right_rot_error
                << '\n';
        }
    }

    // ============================================================================
    // 2. Common FDCC system layer -- initialize once, use for all experiments
    // ============================================================================
    int initFDCC(FDCCContext &sys)
    {
        const char *device_name = HYYRobotBase::get_deviceName(0, nullptr);
        if (device_name == nullptr)
        {
            std::cerr << "FDCC: get_deviceName(0) failed." << std::endl;
            return -105;
        }

        const char *left_arm = get_name_robot_device(device_name, 0);
        const char *right_arm = get_name_robot_device(device_name, 1);
        const char *torso_arm = get_name_robot_device(device_name, 2);
        sys.groups = RobotGroups{torso_arm, left_arm, right_arm, 0};
        sys.dt = get_control_cycle(device_name);

        // 检查HYY控制组及控制周期
        if (sys.groups.torso_name == nullptr || sys.groups.left_arm_name == nullptr || sys.groups.right_arm_name == nullptr)
        {
            std::cerr << "Null robot group name." << std::endl;
            return -101;
        }

        const int torso_dof = get_group_dof(sys.groups.torso_name);
        const int left_dof = get_group_dof(sys.groups.left_arm_name);
        const int right_dof = get_group_dof(sys.groups.right_arm_name);
        if (torso_dof != 3 || left_dof != 7 || right_dof != 7)
        {
            std::cerr << "Unexpected group DOF: torso=" << torso_dof
                      << ", left=" << left_dof
                      << ", right=" << right_dof
                      << " (expected 3/7/7)." << std::endl;
            return -102;
        }

        if (sys.dt <= 0.0)
        {
            std::cerr << "Invalid control period dt=" << sys.dt << std::endl;
            return -100;
        }

        const int model_ret = initFDCCModel(sys.model, sys.idx);
        if (model_ret != 0)
            return model_ret;
        sys.data.reset(new pinocchio::Data(sys.model));

        // 读取真机初始关节状态，顺序统一为：躯干3维、左臂7维、右臂7维
        double q_torso[3] = {0}, q_left[7] = {0}, q_right[7] = {0};
        double dq_torso[3] = {0}, dq_left[7] = {0}, dq_right[7] = {0};
        GetGroupPosition(sys.groups.torso_name, q_torso);
        GetGroupPosition(sys.groups.left_arm_name, q_left);
        GetGroupPosition(sys.groups.right_arm_name, q_right);
        GetGroupVelocity(sys.groups.torso_name, dq_torso);
        GetGroupVelocity(sys.groups.left_arm_name, dq_left);
        GetGroupVelocity(sys.groups.right_arm_name, dq_right);

        for (int i = 0; i < 3; ++i)
        {
            sys.q_real[i] = q_torso[i];
            sys.dq_real[i] = dq_torso[i];
        }
        for (int i = 0; i < 7; ++i)
        {
            sys.q_real[3 + i] = q_left[i];
            sys.dq_real[3 + i] = dq_left[i];
            sys.q_real[10 + i] = q_right[i];
            sys.dq_real[10 + i] = dq_right[i];
        }

        sys.qv = Eigen::VectorXd::Zero(sys.model.nq);
        sys.dqv = Eigen::VectorXd::Zero(sys.model.nv);
        sys.ddqv = Eigen::VectorXd::Zero(sys.model.nv);

        // 将真机 17 维初始关节位置写入 Pinocchio 完整广义坐标 qv
        for (int i = 0; i < 17; ++i)
            sys.qv[sys.idx.active_q_idx[i]] = sys.q_real[i];

        sys.q_nominal = sys.q_real;

        sys.K_left.diagonal() << 100.0, 100.0, 100.0, 150.0, 150.0, 150.0;
        sys.D_left.diagonal() << 20.0, 20.0, 20.0, 30.0, 30.0, 30.0;
        sys.K_right = sys.K_left;
        sys.D_right = sys.D_left;

        sys.K_F_left.diagonal() << 0.2, 0.2, 0.2, 0.2, 0.2, 0.2;
        sys.D_F_left.diagonal() << 0.002, 0.002, 0.002, 0.002, 0.002, 0.002;
        sys.K_F_right = sys.K_F_left;
        sys.D_F_right = sys.D_F_left;

        sys.Kn17.diagonal() << 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2;
        sys.Dn17.diagonal() << 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 1.5, 1.0, 0.8, 0.5, 2.0, 2.0, 2.0, 1.5, 1.0, 0.8, 0.5;

        for (int i = 0; i < 17; ++i)
        {
            sys.q_min_active[i] = sys.model.lowerPositionLimit[sys.idx.active_q_idx[i]];
            sys.q_max_active[i] = sys.model.upperPositionLimit[sys.idx.active_q_idx[i]];
            const double q_range = sys.q_max_active[i] - sys.q_min_active[i];
            sys.valid_limit[i] = std::isfinite(sys.q_min_active[i]) && std::isfinite(sys.q_max_active[i]) && q_range > 1e-6;
            sys.limit_margin[i] = LIMIT_RATIO * q_range;
        }

        sys.F_net_prev_left.setZero();
        sys.F_net_prev_right.setZero();
        sys.dF_filt_left.setZero();
        sys.dF_filt_right.setZero();

        std::cout << "FDCC system initialized. dt =" << sys.dt << " s, groups: left = 0, right = 1, torso = 2." << std::endl;
        return 0;
    }

    int fdccControlStep(FDCCContext &sys, const FDCCStepInput &in, FDCCStepOutput &out)
    {
        // 读取真机当前关节状态
        double q_torso[3] = {0}, q_left[7] = {0}, q_right[7] = {0};
        double dq_torso[3] = {0}, dq_left[7] = {0}, dq_right[7] = {0};
        GetGroupPosition(sys.groups.torso_name, q_torso);
        GetGroupPosition(sys.groups.left_arm_name, q_left);
        GetGroupPosition(sys.groups.right_arm_name, q_right);
        GetGroupVelocity(sys.groups.torso_name, dq_torso);
        GetGroupVelocity(sys.groups.left_arm_name, dq_left);
        GetGroupVelocity(sys.groups.right_arm_name, dq_right);

        for (int i = 0; i < 3; ++i)
        {
            sys.q_real[i] = q_torso[i];
            sys.dq_real[i] = dq_torso[i];
        }
        for (int i = 0; i < 7; ++i)
        {
            sys.q_real[3 + i] = q_left[i];
            sys.dq_real[3 + i] = dq_left[i];
            sys.q_real[10 + i] = q_right[i];
            sys.dq_real[10 + i] = dq_right[i];
        }

        // 真机位置/速度安全检查，触发HYY限位后立即停止本周期控制
        if (IsRobotPositionLimitMargin(sys.groups.torso_name, q_torso, 1) ||
            IsRobotPositionLimitMargin(sys.groups.left_arm_name, q_left, 1) ||
            IsRobotPositionLimitMargin(sys.groups.right_arm_name, q_right, 1))
        {
            std::cerr << "FDCC: upper-body position limit margin reached." << std::endl;
            return -104;
        }

        if (IsRobotVelocityLimitMargin(sys.groups.torso_name, dq_torso, 1) ||
            IsRobotVelocityLimitMargin(sys.groups.left_arm_name, dq_left, 1) ||
            IsRobotVelocityLimitMargin(sys.groups.right_arm_name, dq_right, 1))
        {
            std::cerr << "FDCC: upper-body velocity limit margin reached." << std::endl;
            return -104;
        }

        const pinocchio::SE3 T_left_base = fkineRelativeBase(sys.model, *sys.data, sys.qv, sys.idx.left_ee_frame_id);
        const pinocchio::SE3 T_right_base = fkineRelativeBase(sys.model, *sys.data, sys.qv, sys.idx.right_ee_frame_id);
        out.T_left = in.T_world_base * T_left_base;
        out.T_right = in.T_world_base * T_right_base;

        // 计算完整Jacobian，并提取17个受控关节对应的列
        const Eigen::MatrixXd J_left_full = jacobWorldAlignedFull(sys.model, *sys.data, sys.qv, sys.idx.left_ee_frame_id);
        const Eigen::MatrixXd J_right_full = jacobWorldAlignedFull(sys.model, *sys.data, sys.qv, sys.idx.right_ee_frame_id);
        Matrix6x17d J_left_base;
        Matrix6x17d J_right_base;
        for (int i = 0; i < 17; ++i)
        {
            J_left_base.col(i) = J_left_full.col(sys.idx.active_v_idx[i]);
            J_right_base.col(i) = J_right_full.col(sys.idx.active_v_idx[i]);
        }

        // 将基座坐标系表达的Jacobian旋转到世界坐标系
        Matrix6x17d J_left;
        Matrix6x17d J_right;
        J_left.topRows<3>() = in.T_world_base.rotation() * J_left_base.topRows<3>();
        J_left.bottomRows<3>() = in.T_world_base.rotation() * J_left_base.bottomRows<3>();
        J_right.topRows<3>() = in.T_world_base.rotation() * J_right_base.topRows<3>();
        J_right.bottomRows<3>() = in.T_world_base.rotation() * J_right_base.bottomRows<3>();
        Matrix12x17d J_dual;
        J_dual.topRows<6>() = J_left;
        J_dual.bottomRows<6>() = J_right;

        // 从 Pinocchio 完整速度向量中提取 17 个受控关节速度
        Vector17d dq_active;
        for (int i = 0; i < 17; ++i)
            dq_active[i] = sys.dqv[sys.idx.active_v_idx[i]];
        const Vector6d dx_left = J_left * dq_active;
        const Vector6d dx_right = J_right * dq_active;

        out.xerr_left = poseError6DWorld(in.Td_left, out.T_left);
        out.xerr_right = poseError6DWorld(in.Td_right, out.T_right);
        const Vector6d F_imp_left = computeImpedanceWrenchWorld(out.xerr_left, dx_left, sys.K_left, sys.D_left, in.F_des_left);
        const Vector6d F_imp_right = computeImpedanceWrenchWorld(out.xerr_right, dx_right, sys.K_right, sys.D_right, in.F_des_right);

        // 直接读取左右腕六维力/力矩传感器
        // double F_left[6] = {0}, F_right[6] = {0};
        // GetSensorForce(LEFT_FT_SENSOR_NAME, F_left);
        // GetSensorForce(RIGHT_FT_SENSOR_NAME, F_right);
        // for (int i = 0; i < 6; ++i)
        // {
        //     out.F_sensor_left_local[i] = F_left[i];
        //     out.F_sensor_right_local[i] = F_right[i];
        // }

        out.F_sensor_left_local.setZero();
        out.F_sensor_right_local.setZero();

        if (sys.run_time >= 2.0 && sys.run_time < 6.0)
        {
            out.F_sensor_left_local[2] = -10.0;
        }
        else
        {
            out.F_sensor_left_local.setZero();
        }

        const Vector6d F_sensor_left_world = sensorWrenchLocalToWorldAligned(out.T_left, T_LEFT_EE_FT, out.F_sensor_left_local);
        const Vector6d F_sensor_right_world = sensorWrenchLocalToWorldAligned(out.T_right, T_RIGHT_EE_FT, out.F_sensor_right_local);

        // 合力：阻抗项 + 实测外力 + 期望力
        const Vector6d F_net_left = F_imp_left + F_sensor_left_world + in.F_des_left;
        const Vector6d F_net_right = F_imp_right + F_sensor_right_world + in.F_des_right;
        const Vector6d F_ctrl_left_world = computeControlWrenchPD(F_net_left, sys.F_net_prev_left, sys.dF_filt_left, sys.dt, sys.K_F_left, sys.D_F_left, ALPHA_DF);
        const Vector6d F_ctrl_right_world = computeControlWrenchPD(F_net_right, sys.F_net_prev_right, sys.dF_filt_right, sys.dt, sys.K_F_right, sys.D_F_right, ALPHA_DF);
        sys.F_net_prev_left = F_net_left;
        sys.F_net_prev_right = F_net_right;

        const Vector6d F_ctrl_left_local = wrenchWorldAlignedToLocal(out.T_left, F_ctrl_left_world);
        const Vector6d F_ctrl_right_local = wrenchWorldAlignedToLocal(out.T_right, F_ctrl_right_world);
        const Eigen::VectorXd ddq_task_full = forwardDynamicsDual(sys.model, *sys.data, sys.qv, sys.dqv, F_ctrl_left_local, F_ctrl_right_local, sys.idx.left_ee_frame_id, sys.idx.right_ee_frame_id);
        // 从完整 ABA 加速度结果中提取 17 个受控关节的任务加速度
        Vector17d ddq_task_active;
        for (int i = 0; i < 17; ++i)
            ddq_task_active[i] = ddq_task_full[sys.idx.active_v_idx[i]];

        const Matrix12d A = J_dual * J_dual.transpose() + (LAMBDA_NULL * LAMBDA_NULL) * Matrix12d::Identity();
        const Matrix17x12d J_pinv = J_dual.transpose() * A.ldlt().solve(Matrix12d::Identity());
        const Matrix17d N = Matrix17d::Identity() - J_pinv * J_dual;

        // 从 Pinocchio 完整广义坐标中提取当前 17 维虚拟关节位置
        Vector17d q_active;
        for (int i = 0; i < 17; ++i)
            q_active[i] = sys.qv[sys.idx.active_q_idx[i]];
        const Vector17d ddq_limit_active = computeJointLimitAcceleration(q_active, sys.q_min_active, sys.q_max_active, sys.valid_limit, sys.limit_margin, A_LIM);
        const Vector17d ddq_null_active = -sys.Kn17 * (q_active - sys.q_nominal) - sys.Dn17 * dq_active + ddq_limit_active;
        const Vector17d ddq_active_final = ddq_task_active + N * ddq_null_active;
        const Vector17d dq_active_new = dq_active + ddq_active_final * sys.dt;

        Vector17d q_active_new = q_active;
        for (int i = 0; i < 17; ++i)
        {
            // 限制单周期关节位置增量，避免位置指令突变
            const double q_step = std::clamp(dq_active_new[i] * sys.dt, -Q_STEP_LIMIT, Q_STEP_LIMIT);
            q_active_new[i] += q_step;
        }

        // 将更新后的 17 维虚拟状态直接写回 Pinocchio 完整状态向量
        sys.dqv.setZero();
        sys.ddqv.setZero();
        for (int i = 0; i < 17; ++i)
        {
            sys.qv[sys.idx.active_q_idx[i]] = q_active_new[i];
            sys.dqv[sys.idx.active_v_idx[i]] = dq_active_new[i];
            sys.ddqv[sys.idx.active_v_idx[i]] = ddq_active_final[i];
        }

        // 将17维位置指令拆分后直接下发到躯干、左臂和右臂
        double q_cmd_torso[3], q_cmd_left[7], q_cmd_right[7];
        for (int i = 0; i < 3; ++i)
            q_cmd_torso[i] = q_active_new[i];
        for (int i = 0; i < 7; ++i)
        {
            q_cmd_left[i] = q_active_new[3 + i];
            q_cmd_right[i] = q_active_new[10 + i];
        }

        SetGroupPosition(sys.groups.torso_name, q_cmd_torso);
        SetGroupPosition(sys.groups.left_arm_name, q_cmd_left);
        SetGroupPosition(sys.groups.right_arm_name, q_cmd_right);

        return 0;
    }
    // ============================================================================
    // 3. No-motion self test
    //    No F/T sensor access. No fdccControlStep(). No SetGroupPosition().
    // ============================================================================
    int fdccNoMotionSelfTest()
    {
        int failures = 0;
        int warnings = 0;

        std::cout << "\n================ FDCC NO-MOTION SELF TEST ================\n";
        std::cout << "No F/T sensor access. No motion command will be sent.\n\n";

        // 1. HYY device and robot groups
        const char *device_name = HYYRobotBase::get_deviceName(0, nullptr);
        if (device_name == nullptr)
        {
            std::cerr << "[FAIL] get_deviceName(0) returned nullptr.\n";
            return -1;
        }

        const char *left_arm = get_name_robot_device(device_name, 0);
        const char *right_arm = get_name_robot_device(device_name, 1);
        const char *torso_arm = get_name_robot_device(device_name, 2);
        if (left_arm == nullptr || right_arm == nullptr || torso_arm == nullptr)
        {
            std::cerr << "[FAIL] Failed to get robot group names.\n";
            return -2;
        }

        const int left_dof = get_group_dof(left_arm);
        const int right_dof = get_group_dof(right_arm);
        const int torso_dof = get_group_dof(torso_arm);
        std::cout << "[INFO] group 0 left  = " << left_arm << ", DOF = " << left_dof << '\n';
        std::cout << "[INFO] group 1 right = " << right_arm << ", DOF = " << right_dof << '\n';
        std::cout << "[INFO] group 2 torso = " << torso_arm << ", DOF = " << torso_dof << '\n';
        if (left_dof != 7 || right_dof != 7 || torso_dof != 3)
        {
            std::cerr << "[FAIL] Expected DOF = 7/7/3.\n";
            ++failures;
        }
        else
            std::cout << "[PASS] Robot group DOF check.\n";

        const double dt = get_control_cycle(device_name);
        std::cout << "[INFO] control dt = " << dt << " s\n";
        if (!std::isfinite(dt) || dt <= 0.0)
        {
            std::cerr << "[FAIL] Invalid control period.\n";
            ++failures;
        }
        else if (std::abs(dt - 0.001) > 1e-6)
        {
            std::cout << "[WARN] dt is not 0.001 s.\n";
            ++warnings;
        }
        else
            std::cout << "[PASS] 1 kHz control period.\n";

        // 2. Read real upper-body state directly: torso + left arm + right arm
        double q_torso[3] = {0}, q_left[7] = {0}, q_right[7] = {0};
        double dq_torso[3] = {0}, dq_left[7] = {0}, dq_right[7] = {0};
        GetGroupPosition(torso_arm, q_torso);
        GetGroupPosition(left_arm, q_left);
        GetGroupPosition(right_arm, q_right);
        GetGroupVelocity(torso_arm, dq_torso);
        GetGroupVelocity(left_arm, dq_left);
        GetGroupVelocity(right_arm, dq_right);

        Vector17d q_real = Vector17d::Zero();
        Vector17d dq_real = Vector17d::Zero();
        for (int i = 0; i < 3; ++i)
        {
            q_real[i] = q_torso[i];
            dq_real[i] = dq_torso[i];
        }
        for (int i = 0; i < 7; ++i)
        {
            q_real[3 + i] = q_left[i];
            dq_real[3 + i] = dq_left[i];
            q_real[10 + i] = q_right[i];
            dq_real[10 + i] = dq_right[i];
        }

        std::cout << "\n[INFO] q_real  [T1 T2 T3 L1..L7 R1..R7]:\n"
                  << q_real.transpose() << '\n';
        std::cout << "[INFO] dq_real [T1 T2 T3 L1..L7 R1..R7]:\n"
                  << dq_real.transpose() << '\n';
        if (!q_real.allFinite() || !dq_real.allFinite())
        {
            std::cerr << "[FAIL] q_real/dq_real contains NaN or Inf.\n";
            ++failures;
        }
        else
            std::cout << "[PASS] Real joint state is finite.\n";

        // 3. HYY joint-limit safety check, still no motion command
        if (IsRobotPositionLimitMargin(torso_arm, q_torso, 1) || IsRobotPositionLimitMargin(left_arm, q_left, 1) || IsRobotPositionLimitMargin(right_arm, q_right, 1))
        {
            std::cerr << "[FAIL] Current position is inside HYY limit margin.\n";
            ++failures;
        }
        else
            std::cout << "[PASS] HYY position-limit check.\n";

        if (IsRobotVelocityLimitMargin(torso_arm, dq_torso, 1) || IsRobotVelocityLimitMargin(left_arm, dq_left, 1) || IsRobotVelocityLimitMargin(right_arm, dq_right, 1))
        {
            std::cerr << "[FAIL] Current velocity is inside HYY limit margin.\n";
            ++failures;
        }
        else
            std::cout << "[PASS] HYY velocity-limit check.\n";

        // 4. Pinocchio model and 17 active-joint mapping
        ModelIndex idx;
        pinocchio::Model model;
        const int model_ret = initFDCCModel(model, idx);
        if (model_ret != 0)
        {
            std::cerr << "[FAIL] initFDCCModel(), ret = " << model_ret << '\n';
            return model_ret;
        }
        pinocchio::Data data(model);

        std::cout << "\n[INFO] Pinocchio nq = " << model.nq << ", nv = " << model.nv << ", njoints = " << model.njoints << ", nframes = " << model.nframes << '\n';
        if (idx.active_joint_ids.size() != 17)
        {
            std::cerr << "[FAIL] active_joint_ids size != 17.\n";
            ++failures;
        }
        else
            std::cout << "[PASS] 17 active joints found.\n";

        const std::array<const char *, 17> joint_labels = {"T1", "T2", "T3", "L1", "L2", "L3", "L4", "L5", "L6", "L7", "R1", "R2", "R3", "R4", "R5", "R6", "R7"};
        for (int i = 0; i < 17; ++i)
        {
            std::cout << "  " << joint_labels[i] << ": q_idx = " << idx.active_q_idx[i] << ", v_idx = " << idx.active_v_idx[i] << '\n';
            if (idx.active_q_idx[i] < 0 || idx.active_q_idx[i] >= model.nq || idx.active_v_idx[i] < 0 || idx.active_v_idx[i] >= model.nv)
            {
                std::cerr << "    [FAIL] Invalid q/v index.\n";
                ++failures;
            }
        }

        // 5. Initialize virtual state directly from q_real; no helper function
        Eigen::VectorXd qv = Eigen::VectorXd::Zero(model.nq);
        Eigen::VectorXd dqv = Eigen::VectorXd::Zero(model.nv);
        for (int i = 0; i < 17; ++i)
            qv[idx.active_q_idx[i]] = q_real[i];

        double q_init_max_error = 0.0;
        for (int i = 0; i < 17; ++i)
            q_init_max_error = std::max(q_init_max_error, std::abs(qv[idx.active_q_idx[i]] - q_real[i]));
        std::cout << "[INFO] qv initialization max error = " << q_init_max_error << '\n';
        if (q_init_max_error > 1e-12)
        {
            std::cerr << "[FAIL] qv initialization mismatch.\n";
            ++failures;
        }
        else
            std::cout << "[PASS] qv initialized from q_real.\n";

        // 6. Model joint limits
        std::cout << "\n[INFO] Pinocchio joint limits:\n";
        for (int i = 0; i < 17; ++i)
        {
            const double qmin = model.lowerPositionLimit[idx.active_q_idx[i]];
            const double qmax = model.upperPositionLimit[idx.active_q_idx[i]];
            std::cout << "  " << joint_labels[i] << ": q = " << q_real[i] << ", min = " << qmin << ", max = " << qmax << '\n';
            if (!std::isfinite(qmin) || !std::isfinite(qmax) || qmax <= qmin)
            {
                std::cout << "    [WARN] Invalid/unbounded model limit.\n";
                ++warnings;
            }
            else if (q_real[i] < qmin || q_real[i] > qmax)
            {
                std::cerr << "    [FAIL] Current joint is outside model limit.\n";
                ++failures;
            }
        }

        // 7. Forward kinematics
        const pinocchio::SE3 T_left = fkineRelativeBase(model, data, qv, idx.left_ee_frame_id);
        const pinocchio::SE3 T_right = fkineRelativeBase(model, data, qv, idx.right_ee_frame_id);
        std::cout << "\n[INFO] Left EE position  = " << T_left.translation().transpose() << '\n';
        std::cout << "[INFO] Left EE rotation:\n"
                  << T_left.rotation() << '\n';
        std::cout << "[INFO] Right EE position = " << T_right.translation().transpose() << '\n';
        std::cout << "[INFO] Right EE rotation:\n"
                  << T_right.rotation() << '\n';

        Eigen::VectorXd q_test = Eigen::VectorXd::Zero(model.nq);
        // 17 active joints: T1 T2 T3 L1 L2 L3 L4 L5 L6 L7 R1 R2 R3 R4 R5 R6 R7
        q_test[idx.active_q_idx[0]] = 30 * M_PI / 180.0;  // T1
        q_test[idx.active_q_idx[1]] = -30 * M_PI / 180.0; // T2
        q_test[idx.active_q_idx[2]] = 0.0 * M_PI / 180.0; // T3

        q_test[idx.active_q_idx[3]] = 0.0 * M_PI / 180.0; // L1
        q_test[idx.active_q_idx[4]] = -90 * M_PI / 180.0; // L2
        q_test[idx.active_q_idx[5]] = -90 * M_PI / 180.0; // L3
        q_test[idx.active_q_idx[6]] = -90 * M_PI / 180.0; // L4
        q_test[idx.active_q_idx[7]] = 0.0 * M_PI / 180.0; // L5
        q_test[idx.active_q_idx[8]] = 0.0 * M_PI / 180.0; // L6
        q_test[idx.active_q_idx[9]] = 0.0 * M_PI / 180.0; // L7

        q_test[idx.active_q_idx[10]] = 0.0 * M_PI / 180.0; // R1
        q_test[idx.active_q_idx[11]] = 90 * M_PI / 180.0;  // R2
        q_test[idx.active_q_idx[12]] = 90 * M_PI / 180.0;  // R3
        q_test[idx.active_q_idx[13]] = 90 * M_PI / 180.0;  // R4
        q_test[idx.active_q_idx[14]] = 0.0 * M_PI / 180.0; // R5
        q_test[idx.active_q_idx[15]] = 0.0 * M_PI / 180.0; // R6
        q_test[idx.active_q_idx[16]] = 0.0 * M_PI / 180.0; // R7

        pinocchio::forwardKinematics(model, data, q_test);
        pinocchio::updateFramePlacements(model, data);

        pinocchio::SE3 T_left_test = data.oMf[idx.left_ee_frame_id];
        pinocchio::SE3 T_right_test = data.oMf[idx.right_ee_frame_id];

        std::cout << "\n========== Specify Joint FK ==========" << std::endl;
        std::cout << "[ZERO] Left EE position  = " << T_left_test.translation().transpose() << std::endl;
        std::cout << "[ZERO] Right EE position = " << T_right_test.translation().transpose() << std::endl;
        std::cout << "[ZERO] dx    = " << T_left_test.translation().x() - T_right_test.translation().x() << std::endl;
        std::cout << "[ZERO] y_sum = " << T_left_test.translation().y() + T_right_test.translation().y() << std::endl;
        std::cout << "[ZERO] dz    = " << T_left_test.translation().z() - T_right_test.translation().z() << std::endl;

        std::cout
            << "\n[CHECK] Left/Right joint symmetry:" << std::endl;

        for (int i = 1; i <= 7; ++i)
        {
            std::string left_name = "J-L" + std::to_string(i);
            std::string right_name = "J-R" + std::to_string(i);

            pinocchio::JointIndex left_id = model.getJointId(left_name);
            pinocchio::JointIndex right_id = model.getJointId(right_name);

            Eigen::Vector3d p_left = data.oMi[left_id].translation();
            Eigen::Vector3d p_right = data.oMi[right_id].translation();

            std::cout << "J" << i
                      << "  L = [" << p_left.transpose() << "]"
                      << "  R = [" << p_right.transpose() << "]"
                      << "  dx = " << p_left.x() - p_right.x()
                      << "  y_sum = " << p_left.y() + p_right.y()
                      << "  dz = " << p_left.z() - p_right.z()
                      << std::endl;
        }

        // 8. Jacobian: obtain full Jacobian and directly extract the 17 active columns here
        const Eigen::MatrixXd J_left_full = jacobWorldAlignedFull(model, data, q_test, idx.left_ee_frame_id);
        const Eigen::MatrixXd J_right_full = jacobWorldAlignedFull(model, data, q_test, idx.right_ee_frame_id);
        Matrix6x17d J_left;
        Matrix6x17d J_right;
        for (int i = 0; i < 17; ++i)
        {
            J_left.col(i) = J_left_full.col(idx.active_v_idx[i]);
            J_right.col(i) = J_right_full.col(idx.active_v_idx[i]);
        }

        Matrix12x17d J_dual;
        J_dual.topRows<6>() = J_left;
        J_dual.bottomRows<6>() = J_right;
        if (!J_left.allFinite() || !J_right.allFinite())
        {
            std::cerr << "[FAIL] Jacobian contains NaN or Inf.\n";
            ++failures;
        }
        else
            std::cout << "[PASS] Left/right 6x17 Jacobians are finite.\n";

        Eigen::JacobiSVD<Matrix12x17d> svd(J_dual);
        std::cout << "[INFO] J_dual singular values = " << svd.singularValues().transpose() << '\n';
        int rank = 0;
        for (int i = 0; i < svd.singularValues().size(); ++i)
            if (svd.singularValues()[i] > 1e-6)
                ++rank;
        std::cout << "[INFO] J_dual numerical rank = " << rank << " / 12\n";
        if (rank < 10)
        {
            std::cout << "[WARN] J_dual rank is relatively low at current posture.\n";
            ++warnings;
        }

        // 9. Jacobian finite-difference verification against FK
        const double eps_fd = 1e-7;
        double max_left_fd_error = 0.0;
        double max_right_fd_error = 0.0;
        int max_left_joint = -1;
        int max_right_joint = -1;
        for (int i = 0; i < 17; ++i)
        {
            Eigen::VectorXd q_eps = q_test;
            q_eps[idx.active_q_idx[i]] += eps_fd;
            const pinocchio::SE3 T_left_eps = fkineRelativeBase(model, data, q_eps, idx.left_ee_frame_id);
            const pinocchio::SE3 T_right_eps = fkineRelativeBase(model, data, q_eps, idx.right_ee_frame_id);

            Vector6d left_fd = Vector6d::Zero();
            left_fd.head<3>() = (T_left_eps.translation() - T_left_test.translation()) / eps_fd;
            left_fd.tail<3>() = pinocchio::log3(T_left_eps.rotation() * T_left_test.rotation().transpose()) / eps_fd;
            const double left_err = (left_fd - J_left.col(i)).norm();
            if (left_err > max_left_fd_error)
            {
                max_left_fd_error = left_err;
                max_left_joint = i;
            }

            Vector6d right_fd = Vector6d::Zero();
            right_fd.head<3>() = (T_right_eps.translation() - T_right_test.translation()) / eps_fd;
            right_fd.tail<3>() = pinocchio::log3(T_right_eps.rotation() * T_right_test.rotation().transpose()) / eps_fd;
            const double right_err = (right_fd - J_right.col(i)).norm();
            if (right_err > max_right_fd_error)
            {
                max_right_fd_error = right_err;
                max_right_joint = i;
            }
        }

        std::cout << "\n[INFO] Jacobian1 finite-difference max error: left = " << max_left_fd_error << " at " << (max_left_joint >= 0 ? joint_labels[max_left_joint] : "N/A") << ", right = " << max_right_fd_error << " at " << (max_right_joint >= 0 ? joint_labels[max_right_joint] : "N/A") << '\n';
        if (max_left_fd_error > 1e-4 || max_right_fd_error > 1e-4)
        {
            std::cerr << "[FAIL] Jacobian finite-difference check failed.\n";
            ++failures;
        }
        else
            std::cout << "[PASS] Jacobian agrees with FK finite difference.\n";

        // 10. ABA zero-input check
        const Vector6d zero6 = Vector6d::Zero();
        const Eigen::VectorXd ddq_zero = forwardDynamicsDual(model, data, q_test, dqv, zero6, zero6, idx.left_ee_frame_id, idx.right_ee_frame_id);
        std::cout << "\n[INFO] ABA zero-input ||ddq|| = " << ddq_zero.norm() << '\n';
        if (!ddq_zero.allFinite())
        {
            std::cerr << "[FAIL] ABA returned NaN or Inf.\n";
            ++failures;
        }
        else if (ddq_zero.norm() > 1e-8)
        {
            std::cout << "[WARN] ABA zero-input acceleration is not near zero.\n";
            ++warnings;
        }
        else
            std::cout << "[PASS] ABA zero-input result is near zero.\n";

        // 11. Damped pseudoinverse / null-space numerical check
        const Matrix12d A = J_dual * J_dual.transpose() + (LAMBDA_NULL * LAMBDA_NULL) * Matrix12d::Identity();
        const Matrix17x12d J_pinv = J_dual.transpose() * A.ldlt().solve(Matrix12d::Identity());
        const Matrix17d N = Matrix17d::Identity() - J_pinv * J_dual;
        const double null_residual = (J_dual * N).norm();
        std::cout << "[INFO] ||J_dual * N|| = " << null_residual << '\n';
        if (!A.allFinite() || !J_pinv.allFinite() || !N.allFinite())
        {
            std::cerr << "[FAIL] Pseudoinverse/null-space contains NaN or Inf.\n";
            ++failures;
        }
        else if (null_residual > 0.1)
        {
            std::cout << "[WARN] Damped null-space residual is relatively large.\n";
            ++warnings;
        }
        else
            std::cout << "[PASS] Pseudoinverse/null-space numerical check.\n";

        std::cout << "\n===========================================================\n";
        std::cout << "SELF TEST SUMMARY: failures = " << failures << ", warnings = " << warnings << '\n';
        if (failures == 0)
            std::cout << "RESULT: PASS. No motion command was sent.\n";
        else
            std::cout << "RESULT: FAIL. No motion command was sent.\n";
        std::cout << "===========================================================\n";

        return failures == 0 ? 0 : -1000 - failures;
    }
    // ============================================================================
    // 4. Experiment-specific layer
    //    Future experiments should mainly add/modify functions in this section.
    // ============================================================================
    int runExperiment1_1(FDCCContext &sys, const std::string &csv_path)
    {
        FDCCStepInput input;
        input.T_world_base = pinocchio::SE3::Identity(); // Experiment 1-1: fixed base. PSG experiments update this every cycle.
        const pinocchio::SE3 T_left_0 = input.T_world_base * fkineRelativeBase(sys.model, *sys.data, sys.qv, sys.idx.left_ee_frame_id);
        const pinocchio::SE3 T_right_0 = input.T_world_base * fkineRelativeBase(sys.model, *sys.data, sys.qv, sys.idx.right_ee_frame_id);
        // 保存实验开始时的末端位姿，作为恒定参考位姿
        const pinocchio::SE3 T_left_ref = T_left_0;
        const pinocchio::SE3 T_right_ref = T_right_0;
        input.Td_left = T_left_0;
        input.Td_right = T_right_0;
        input.F_des_left.setZero();
        input.F_des_right.setZero();

        RTimer timer;
        initUserTimer(&timer, 1, 1);

        std::vector<FDCCLogRow> log;
        log.reserve(20000);
        int experiment_ret = 0;

        while (robot_ok())
        {
            userTimer(&timer);
            sys.run_time += sys.dt;
            FDCCStepOutput output;
            const int ret = fdccControlStep(sys, input, output);
            if (ret != 0)
            {
                experiment_ret = ret;
                break;
            }

            FDCCLogRow row;
            row.t = sys.run_time;
            row.F_left_sensor_local = output.F_sensor_left_local;
            row.F_right_sensor_local = output.F_sensor_right_local;
            row.left_disp_local = T_left_ref.rotation().transpose() * (output.T_left.translation() - T_left_ref.translation());
            row.right_disp_local = T_right_ref.rotation().transpose() * (output.T_right.translation() - T_right_ref.translation());
            row.left_pos_error = output.xerr_left.head<3>().norm();
            row.right_pos_error = output.xerr_right.head<3>().norm();
            row.left_rot_error = output.xerr_left.tail<3>().norm();
            row.right_rot_error = output.xerr_right.tail<3>().norm();
            log.push_back(row);
        }

        // 实验结束后统一保存本次实验数据
        saveLogCSV(csv_path, log);
        std::cout << "FDCC experiment 1-1 stopped. CSV saved to: " << csv_path << std::endl;

        return experiment_ret;
    }

    // ============================================================================
    // Public entry for main.cpp
    // ============================================================================
    int fdccHRIExperiment1_1Start()
    {
        ClearDeviceError();
        IMPORTJOINT(R0_PTEST);
        IMPORTJOINT(R1_PTEST);
        IMPORTJOINT(R2_PTEST);
        IMPORTSPEED(v1);

        // Move to initial pose
        RobotPower(0);
        RobotPower(1);
        RobotPower(2);
        CustomMoveA(R0_PTEST, v1, R1_PTEST, v1, R2_PTEST, v1);
        FDCCContext sys;
        const int ret = initFDCC(sys);
        if (ret != 0)
            return ret;
        return runExperiment1_1(sys, "fdcc_hri_real_data.csv");
    }

} // namespace fdcc
