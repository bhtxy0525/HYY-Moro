#pragma once

/**
 * @file fdcc_hri_controller.h
 * @brief 17自由度上肢FDCC控制器及实验接口
 */

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/spatial/se3.hpp>

#include <Eigen/Dense>

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace fdcc
{

    using Vector3d = Eigen::Matrix<double, 3, 1>;
    using Vector6d = Eigen::Matrix<double, 6, 1>;
    using Vector17d = Eigen::Matrix<double, 17, 1>;
    using Matrix6d = Eigen::Matrix<double, 6, 6>;
    using Matrix17d = Eigen::Matrix<double, 17, 17>;
    using Matrix6x17d = Eigen::Matrix<double, 6, 17>;
    using Matrix12x17d = Eigen::Matrix<double, 12, 17>;
    using Matrix12d = Eigen::Matrix<double, 12, 12>;
    using Matrix17x12d = Eigen::Matrix<double, 17, 12>;

    // ============================================================================
    // 固定配置
    // ============================================================================
    extern const std::string FDCC_MJCF_PATH;
    extern const std::string LEFT_EE_FRAME_NAME;
    extern const std::string RIGHT_EE_FRAME_NAME;

    inline constexpr double LIMIT_RATIO = 0.05;
    inline constexpr double A_LIM = 1.0;
    inline constexpr double LAMBDA_NULL = 0.02;
    inline constexpr double ALPHA_DF = 0.1;
    inline constexpr double Q_STEP_LIMIT = 0.001;
    inline constexpr double EPS = 1e-8;

    // 左右腕六维力传感器名称
    inline constexpr char LEFT_FT_SENSOR_NAME[] = "left_sensor";
    inline constexpr char RIGHT_FT_SENSOR_NAME[] = "right_sensor";

    // 左右腕力传感器相对于末端法兰的固定安装变换
    inline const pinocchio::SE3 T_LEFT_EE_FT(
        Eigen::Matrix3d::Identity(),
        Eigen::Vector3d(0.0, 0.0, 0.0));

    inline const pinocchio::SE3 T_RIGHT_EE_FT(
        Eigen::Matrix3d::Identity(),
        Eigen::Vector3d(0.0, 0.0, 0.0));

    // ============================================================================
    // 公共数据结构
    // ============================================================================
    struct RobotGroups
    {
        const char *torso_name = nullptr;
        const char *left_arm_name = nullptr;
        const char *right_arm_name = nullptr;
        int timer_robot_index = 0;
    };

    struct ModelIndex
    {
        std::vector<pinocchio::JointIndex> torso_joint_ids;
        std::vector<pinocchio::JointIndex> left_arm_joint_ids;
        std::vector<pinocchio::JointIndex> right_arm_joint_ids;
        std::vector<pinocchio::JointIndex> active_joint_ids;
        std::array<int, 17> active_q_idx{};
        std::array<int, 17> active_v_idx{};
        pinocchio::FrameIndex left_ee_frame_id{};
        pinocchio::FrameIndex right_ee_frame_id{};
    };

    struct FDCCContext
    {
        RobotGroups groups;
        pinocchio::Model model;
        std::unique_ptr<pinocchio::Data> data;
        ModelIndex idx;

        double dt = 0.001;
        double run_time = 0.0;

        Vector17d q_real = Vector17d::Zero();
        Vector17d dq_real = Vector17d::Zero();
        Eigen::VectorXd qv;
        Eigen::VectorXd dqv;
        Eigen::VectorXd ddqv;

        Matrix6d K_left = Matrix6d::Zero();
        Matrix6d D_left = Matrix6d::Zero();
        Matrix6d K_right = Matrix6d::Zero();
        Matrix6d D_right = Matrix6d::Zero();
        Matrix6d K_F_left = Matrix6d::Zero();
        Matrix6d D_F_left = Matrix6d::Zero();
        Matrix6d K_F_right = Matrix6d::Zero();
        Matrix6d D_F_right = Matrix6d::Zero();

        Matrix17d Kn17 = Matrix17d::Zero();
        Matrix17d Dn17 = Matrix17d::Zero();
        Vector17d q_nominal = Vector17d::Zero();

        Vector17d q_min_active = Vector17d::Zero();
        Vector17d q_max_active = Vector17d::Zero();
        Vector17d limit_margin = Vector17d::Zero();
        std::array<bool, 17> valid_limit{};

        pinocchio::SE3 T_left_ee_ft = pinocchio::SE3::Identity();  // left_sensor mount place
        pinocchio::SE3 T_right_ee_ft = pinocchio::SE3::Identity(); // right_sensor mount place

        Vector6d F_net_prev_left = Vector6d::Zero();
        Vector6d F_net_prev_right = Vector6d::Zero();
        Vector6d dF_filt_left = Vector6d::Zero();
        Vector6d dF_filt_right = Vector6d::Zero();
    };

    struct FDCCStepInput
    {
        pinocchio::SE3 T_world_base = pinocchio::SE3::Identity();
        pinocchio::SE3 Td_left = pinocchio::SE3::Identity();
        pinocchio::SE3 Td_right = pinocchio::SE3::Identity();
        Vector6d F_des_left = Vector6d::Zero();
        Vector6d F_des_right = Vector6d::Zero();
    };

    struct FDCCStepOutput
    {
        pinocchio::SE3 T_left = pinocchio::SE3::Identity();
        pinocchio::SE3 T_right = pinocchio::SE3::Identity();
        Vector6d F_sensor_left_local = Vector6d::Zero();
        Vector6d F_sensor_right_local = Vector6d::Zero();
        Vector6d xerr_left = Vector6d::Zero();
        Vector6d xerr_right = Vector6d::Zero();
    };

    struct FDCCLogRow
    {
        double t = 0.0;
        Vector6d F_left_sensor_local = Vector6d::Zero();
        Vector6d F_right_sensor_local = Vector6d::Zero();
        Vector3d left_disp_local = Vector3d::Zero();
        Vector3d right_disp_local = Vector3d::Zero();
        double left_pos_error = 0.0;
        double right_pos_error = 0.0;
        double left_rot_error = 0.0;
        double right_rot_error = 0.0;
    };

    // ============================================================================
    // FDCC算法与模型函数
    // ============================================================================

    /**
     * @brief 计算向量a在向量b方向上的投影分量
     * @param a 待投影向量
     * @param b 投影方向向量
     * @return a在b方向上的平行分量；当b接近零向量时返回零向量
     */
    Vector3d projectParallel(const Vector3d &a, const Vector3d &b);

    /**
     * @brief 加载MJCF模型并建立17个受控关节及双末端执行器的索引映射
     * @param model 输出的Pinocchio机器人模型
     * @param idx 输出的受控关节及末端frame索引
     * @return 0:初始化成功；负数:模型、关节或末端frame配置错误
     */
    int initFDCCModel(pinocchio::Model &model, ModelIndex &idx);

    /**
     * @brief 计算末端执行器相对于机器人基座坐标系的位姿
     * @param model Pinocchio机器人模型
     * @param data Pinocchio运行数据
     * @param q 完整广义坐标
     * @param ee_frame_id 末端执行器frame索引
     * @return 基座坐标系下的末端SE3位姿
     */
    pinocchio::SE3 fkineRelativeBase(const pinocchio::Model &model,
                                     pinocchio::Data &data,
                                     const Eigen::VectorXd &q,
                                     pinocchio::FrameIndex ee_frame_id);

    /**
     * @brief 计算末端执行器LOCAL_WORLD_ALIGNED形式的完整Jacobian。
     * @param model Pinocchio机器人模型
     * @param data Pinocchio运行数据
     * @param q 完整广义坐标
     * @param ee_frame_id 末端执行器frame索引
     * @return 6×nv完整Jacobian矩阵
     */
    Eigen::MatrixXd jacobWorldAlignedFull(const pinocchio::Model &model,
                                          pinocchio::Data &data,
                                          const Eigen::VectorXd &q,
                                          pinocchio::FrameIndex ee_frame_id);

    /**
     * @brief 计算世界坐标系下的6维末端位姿误差
     * @param Td 期望末端位姿
     * @param T 当前末端位姿
     * @return 前3维为位置误差，后3维为旋转误差
     */
    Vector6d poseError6DWorld(const pinocchio::SE3 &Td, const pinocchio::SE3 &T);

    /**
     * @brief 使用力旋量的对偶伴随变换将力/力矩从坐标系B转换到坐标系A
     * @param T_A_B 坐标系B相对于坐标系A的SE3变换
     * @param F_B 坐标系B中表达的6维力/力矩
     * @return 坐标系A中表达的6维力/力矩
     */
    Vector6d transformWrenchDualAdjoint(const pinocchio::SE3 &T_A_B, const Vector6d &F_B);

    /**
     * @brief 将力传感器局部坐标系中的测量值转换为世界对齐坐标系中的力/力矩
     * @param T_world_ee 末端执行器在世界坐标系中的位姿
     * @param T_ee_ft 力传感器相对于末端执行器的安装变换
     * @param F_sensor_local 力传感器局部坐标系中的6维测量值
     * @return 世界对齐坐标系中的6维力/力矩
     */
    Vector6d sensorWrenchLocalToWorldAligned(const pinocchio::SE3 &T_world_ee,
                                             const pinocchio::SE3 &T_ee_ft,
                                             const Vector6d &F_sensor_local);

    /**
     * @brief 将世界对齐坐标系中的力/力矩旋转到末端执行器局部坐标系
     * @param T_world_ee 末端执行器在世界坐标系中的位姿
     * @param F_world 世界对齐坐标系中的6维力/力矩
     * @return 末端执行器局部坐标系中的6维力/力矩
     */
    Vector6d wrenchWorldAlignedToLocal(const pinocchio::SE3 &T_world_ee,
                                       const Vector6d &F_world);

    /**
     * @brief 根据末端位姿误差和速度计算任务空间阻抗力
     * @param xerr 6维位姿误差
     * @param dx 6维末端速度
     * @param K 任务空间刚度矩阵
     * @param D 任务空间阻尼矩阵
     * @param F_des 期望交互力，用于去除阻抗力在期望力方向上的分量
     * @return 世界坐标系表达的6维阻抗力
     */
    Vector6d computeImpedanceWrenchWorld(const Vector6d &xerr,
                                         const Vector6d &dx,
                                         const Matrix6d &K,
                                         const Matrix6d &D,
                                         const Vector6d &F_des);

    /**
     * @brief 根据合力误差及其变化率计算FDCC控制力
     * @param F_net 当前6维合力误差
     * @param F_net_prev 上一周期6维合力误差
     * @param dF_filt 输入输出参数，滤波后的合力变化率
     * @param dt 控制周期，单位s
     * @param K_F 力比例增益矩阵
     * @param D_F 力微分增益矩阵
     * @param alpha 一阶低通滤波系数
     * @return FDCC输出的6维控制力
     */
    Vector6d computeControlWrenchPD(const Vector6d &F_net,
                                    const Vector6d &F_net_prev,
                                    Vector6d &dF_filt,
                                    double dt,
                                    const Matrix6d &K_F,
                                    const Matrix6d &D_F,
                                    double alpha);

    /**
     * @brief 将左右末端控制力作为外力输入Pinocchio ABA，计算完整广义加速度
     * @param model Pinocchio机器人模型
     * @param data Pinocchio运行数据
     * @param q 完整虚拟关节位置
     * @param dq 完整虚拟关节速度
     * @param F_left_ee_local 左末端局部坐标系中的控制力
     * @param F_right_ee_local 右末端局部坐标系中的控制力
     * @param left_frame_id 左末端frame索引
     * @param right_frame_id 右末端frame索引
     * @return Pinocchio完整广义加速度
     */
    Eigen::VectorXd forwardDynamicsDual(const pinocchio::Model &model,
                                        pinocchio::Data &data,
                                        const Eigen::VectorXd &q,
                                        const Eigen::VectorXd &dq,
                                        const Vector6d &F_left_ee_local,
                                        const Vector6d &F_right_ee_local,
                                        pinocchio::FrameIndex left_frame_id,
                                        pinocchio::FrameIndex right_frame_id);

    /**
     * @brief 计算17个受控关节的关节限位恢复加速度
     * @param q_active 当前17维关节位置
     * @param q_min 17维关节下限
     * @param q_max 17维关节上限
     * @param valid 各关节限位数据是否有效
     * @param margin 各关节安全边界宽度
     * @param a_lim 最大关节限位恢复加速度
     * @return 17维关节限位恢复加速度
     */
    Vector17d computeJointLimitAcceleration(const Vector17d &q_active,
                                            const Vector17d &q_min,
                                            const Vector17d &q_max,
                                            const std::array<bool, 17> &valid,
                                            const Vector17d &margin,
                                            double a_lim);

    // ============================================================================
    // 实验数据记录
    // ============================================================================

    /**
     * @brief 将FDCC实验过程中记录的数据保存为CSV文件
     * @param path CSV文件保存路径
     * @param log 实验数据记录，每个元素对应一个控制周期
     */
    void saveLogCSV(const std::string &path, const std::vector<FDCCLogRow> &log);

    // ============================================================================
    // FDCC控制器接口
    // ============================================================================

    /**
     * @brief 初始化FDCC真机控制系统，包括HYY控制组、Pinocchio模型、控制周期和控制参数
     * @param ctx FDCC运行上下文，初始化结果写入该对象
     * @return 0:初始化成功；负数:初始化失败
     */
    int initFDCC(FDCCContext &ctx);

    /**
     * @brief 执行一个FDCC控制周期
     * @param ctx FDCC运行上下文及控制参数
     * @param in 当前周期的期望末端位姿、期望力和世界-底盘变换
     * @param out 输出当前末端位姿、传感器数据及位姿误差
     * @return 0:本周期执行成功；负数:安全检查或控制流程失败
     */
    int fdccControlStep(FDCCContext &ctx, const FDCCStepInput &in, FDCCStepOutput &out);

    /**
     * @brief FDCC 无运动自检
     * @return 0: 检查通过；非0: 存在异常
     */
    int fdccNoMotionSelfTest();

    // ============================================================================
    // 实验接口
    // ============================================================================

    /**
     * @brief 执行实验1-1的真机FDCC循环并记录实验数据
     * @param ctx 已初始化的FDCC运行上下文
     * @param csv_path 实验数据CSV保存路径
     * @return 0:正常结束；负数:实验过程中发生错误
     */
    int runExperiment1_1(FDCCContext &ctx, const std::string &csv_path);

    /**
     * @brief 实验1-1对外启动接口，可直接在main函数中调用
     * @return 0:实验正常结束；负数:初始化或控制过程失败
     */
    int fdccHRIExperiment1_1Start();

    // 后续实验可按相同形式增加：
    // int runExperiment1_2(FDCCContext &ctx, const std::string &csv_path);
    // int fdccHRIExperiment1_2Start();

} // namespace fdcc
