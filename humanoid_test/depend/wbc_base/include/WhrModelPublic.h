#pragma once

#include <memory>
#include <Eigen/Dense>

/// @defgroup Kinematics 正运动学
/// @defgroup Jacobian 雅可比
/// @defgroup IK 逆运动学
/// @defgroup Utility 工具函数

/**
 * @brief 轮式人行机器人全身运动学与动力学接口
 *
 * 提供以下功能：
 * - 正运动学
 * - 逆运动学
 * - 雅可比矩阵
 * - 可操作度
 * - ZMP监测
 * - 自碰撞监测
 */

class WhrModel
{
public:
    WhrModel();
    ~WhrModel();

    WhrModel(const WhrModel &) = delete;
    WhrModel &operator=(const WhrModel &) = delete;
    WhrModel(WhrModel &&) noexcept;
    WhrModel &operator=(WhrModel &&) noexcept;

    // ---------------- 正运动学 ----------------
    /**
     * @brief 获取底盘正向运动学
     * @ingroup Kinematics
     * @param q_base 底盘X向位移和转角
     * @param RT 4×4齐次变换矩阵
     * @return
     */
    void fkineBase(const double *q_base, Eigen::Matrix4d &RT) const;
    /**
     * @brief 获取左臂基座到左臂末端正向运动学
     * @ingroup Kinematics
     * @param q_left 左臂当前关节角，维度7×1，单位rad
     * @param RT 4×4齐次变换矩阵
     * @return
     */
    void fkineL(const double *q_left, Eigen::Matrix4d &RT) const;
    /**
     * @brief 获取右臂基座到右臂末端正向运动学
     * @ingroup Kinematics
     * @param q_right 右臂当前关节角，维度7×1，单位rad
     * @param RT 4×4齐次变换矩阵
     * @return
     */
    void fkineR(const double *q_right, Eigen::Matrix4d &RT) const;
    /**
     * @brief 获取躯干基座到躯干末端正向运动学
     * @ingroup Kinematics
     * @param q_torso 躯干当前关节角，维度3×1，单位rad
     * @param RT 4×4齐次变换矩阵
     * @return
     */
    void fkineT(const double *q_torso, Eigen::Matrix4d &RT) const;
    /**
     * @brief 获取躯干基座到左臂末端正向运动学
     * @ingroup Kinematics
     * @param q_torso_left 躯干和左臂当前关节角，维度10×1，单位rad
     * @param RT 4×4齐次变换矩阵
     * @return
     */
    void fkineTL(const double *q_torso_left, Eigen::Matrix4d &RT) const;
    /**
     * @brief 获取躯干基座到右臂末端正向运动学
     * @ingroup Kinematics
     * @param q_torso_right 躯干和右臂当前关节角，维度10×1，单位rad
     * @param RT 4×4齐次变换矩阵
     * @return
     */
    void fkineTR(const double *q_torso_right, Eigen::Matrix4d &RT) const;
    /**
     * @brief 获取躯干三关节到头部正向运动学
     * @ingroup Kinematics
     * @param q_hand 头部当前关节角，维度2×1，单位rad
     * @param RT 4×4齐次变换矩阵
     * @return
     */
    void fkineH(const double *q_head, Eigen::Matrix4d &RT) const;
    /**
     * @brief 获取全身正向运动学
     * @ingroup Kinematics
     * @param q_base_torso_left_right 底盘、躯干、左臂和右臂关节角，维度19×1，单位rad
     * @param index 0,左臂;1,右臂
     * @param RT 4×4齐次变换矩阵
     * @return
     */
    void fkine(const double *q_base_torso_left_right, int index, Eigen::Matrix4d &RT) const;

    // ---------------- 雅可比矩阵 ----------------
    /**
     * @brief 获取左臂雅可比矩阵，相对左臂末端
     * @ingroup Jacobian
     * @param q_left 左臂当前关节角，维度7×1，单位rad
     * @param Jl 6×7齐次变换矩阵
     * @return
     */
    void jacobeL(const double *q_left, Eigen::Matrix<double, 6, 7> &Je) const;
    /**
     * @brief 获取左臂雅可比矩阵，相对左臂基座
     * @ingroup Jacobian
     * @param q_left 左臂当前关节角，维度7×1，单位rad
     * @param Jl 6×7齐次变换矩阵
     * @return
     */
    void jacob0L(const double *q_left, Eigen::Matrix<double, 6, 7> &Jl) const;
    /**
     * @brief 获取右臂雅可比矩阵，相对右臂末端
     * @ingroup Jacobian
     * @param q_right 右臂当前关节角，维度7×1，单位rad
     * @param Jr 6×7齐次变换矩阵
     * @return
     */
    void jacobeR(const double *q_right, Eigen::Matrix<double, 6, 7> &Je) const;
    /**
     * @brief 获取右臂雅可比矩阵，相对右臂基座
     * @ingroup Jacobian
     * @param q_right 右臂当前关节角，维度7×1，单位rad
     * @param Jr 6×7齐次变换矩阵
     * @return
     */
    void jacob0R(const double *q_right, Eigen::Matrix<double, 6, 7> &Jr) const;
    /**
     * @brief 获取左臂支链雅可比矩阵，相对左臂末端
     * @ingroup Jacobian
     * @param q_left 左臂当前关节角，维度10×1，单位rad
     * @param Jl 6×10齐次变换矩阵
     * @return
     */
    void jacobeTL(const double *q_torso_left, Eigen::Matrix<double, 6, 10> &Je) const;
    /**
     * @brief 获取左臂支链雅可比矩阵，相对躯干基座
     * @ingroup Jacobian
     * @param q_left 左臂当前关节角，维度10×1，单位rad
     * @param Jl 6×10齐次变换矩阵
     * @return
     */
    void jacob0TL(const double *q_torso_left, Eigen::Matrix<double, 6, 10> &Jl) const;
    /**
     * @brief 获取右臂支链雅可比矩阵，相对右臂末端
     * @ingroup Jacobian
     * @param q_torso_right 右臂当前关节角，维度10×1，单位rad
     * @param Jr 6×10齐次变换矩阵
     * @return
     */
    void jacobeTR(const double *q_torso_right, Eigen::Matrix<double, 6, 10> &Je) const;
    /**
     * @brief 获取右臂支链雅可比矩阵，相对躯干基座
     * @ingroup Jacobian
     * @param q_torso_right 右臂+躯干当前关节角，维度10×1，单位rad
     * @param Jr 6×10齐次变换矩阵
     * @return
     */
    void jacob0TR(const double *q_torso_right, Eigen::Matrix<double, 6, 10> &Jr) const;
    /**
     * @brief 获取躯干支链雅可比矩阵，相对躯干末端
     * @ingroup Jacobian
     * @param q_torso 躯干当前关节角，维度3×1，单位rad
     * @param Jt 6×3齐次变换矩阵
     * @return
     */
    void jacobeT(const double *q_torso, Eigen::Matrix<double, 6, 3> &Je) const;
    /**
     * @brief 获取躯干支链雅可比矩阵，相对躯干基座
     * @ingroup Jacobian
     * @param q_torso 躯干当前关节角，维度3×1，单位rad
     * @param Jt 6×3齐次变换矩阵
     * @return
     */
    void jacob0T(const double *q_torso, Eigen::Matrix<double, 6, 3> &Jt) const;
    /**
     * @brief 获取左臂支链雅可比矩阵，相对左臂末端(基坐标系为底盘)
     * @ingroup Jacobian
     * @param q_base_torso_left 底盘+躯干+左臂当前关节角，维度12×1，单位rad
     * @param Jl 6×12齐次变换矩阵
     * @return
     */
    void jacobeCL(const double *q_base_torso_left, Eigen::Matrix<double, 6, 12> &Je) const;
    /**
     * @brief 获取左臂支链雅可比矩阵，相对底盘基座(基坐标系为底盘)
     * @ingroup Jacobian
     * @param q_base_torso_left 底盘+躯干+左臂当前关节角，维度12×1，单位rad
     * @param Jl 6×12齐次变换矩阵
     * @return
     */
    void jacob0CL(const double *q_base_torso_left, Eigen::Matrix<double, 6, 12> &Jl) const;
    /**
     * @brief 获取右臂支链雅可比矩阵，相对右臂末端(基坐标系为底盘)
     * @ingroup Jacobian
     * @param q_base_torso_right 底盘+躯干+右臂当前关节角，维度12×1，单位rad
     * @param Jr 6×12齐次变换矩阵
     * @return
     */
    void jacobeCR(const double *q_base_torso_right, Eigen::Matrix<double, 6, 12> &Je) const;
    /**
     * @brief 获取右臂支链雅可比矩阵，相对底盘基座(基坐标系为底盘)
     * @ingroup Jacobian
     * @param q_base_torso_right 底盘+躯干+右臂当前关节角，维度12×1，单位rad
     * @param Jr 6×12齐次变换矩阵
     * @return
     */
    void jacob0CR(const double *q_base_torso_right, Eigen::Matrix<double, 6, 12> &Jr) const;
    /**
     * @brief 获取躯干支链雅可比矩阵，相对躯干末端(基坐标系为底盘)
     * @ingroup Jacobian
     * @param q_base_torso 底盘+躯干当前关节角，维度5×1，单位rad
     * @param Jt 6×5齐次变换矩阵
     * @return
     */
    void jacobeCT(const double *q_base_torso, Eigen::Matrix<double, 6, 5> &Je) const;
    /**
     * @brief 获取躯干支链雅可比矩阵，相对底盘基座(基坐标系为底盘)
     *  @ingroup Jacobian
     * @param q_base_torso 底盘+躯干当前关节角，维度5×1，单位rad
     * @param Jt 6×5齐次变换矩阵
     * @return
     */
    void jacob0CT(const double *q_base_torso, Eigen::Matrix<double, 6, 5> &Jt) const;
    /**
     * @brief 获取轮式人形机器人整体雅可比矩阵，相对末端
     * @ingroup Jacobian
     * @param q 底盘+躯干+双臂当前关节角，维度19×1，单位rad
     * @param Je 12×19齐次变换矩阵
     * @return
     */
    void jacobe(const double *q, Eigen::Matrix<double, 12, 19> &Je) const;
    /**
     * @brief 获取轮式人形机器人整体雅可比矩阵，相对底盘基座
     * @ingroup Jacobian
     * @param q 底盘+躯干+双臂当前关节角，维度19×1，单位rad
     * @param J0 12×19齐次变换矩阵
     * @return
     */
    void jacob0(const double *q, Eigen::Matrix<double, 12, 19> &J0) const;
    /**
     * @brief 获取上肢可比矩阵，相对躯干基座
     * @ingroup Jacobian
     * @param q 上肢当前关节角，维度17×1，单位rad
     * @param Ju 12×17齐次变换矩阵
     * @return
     */
    void jacob0U(const double *q, Eigen::Matrix<double, 12, 17> &Ju) const;
    /**
     * @brief 获取底盘雅可比矩阵，相对底盘基座
     * @ingroup Jacobian
     * @param q_base 底盘当前关节角，维度2×1，单位rad/m
     * @param Jb 6×2齐次变换矩阵
     * @return
     */
    void jacob0B(const double *q_base, Eigen::Matrix<double, 6, 2> &Jb) const;

    // ---------------- 逆运动学 ----------------
    /**
     * @brief 上肢逆运动学
     * @ingroup IK
     * @param q_torso_left_right 当前躯干-左臂-右臂关节角
     * @param DesiredTL 左臂期望位姿
     * @param DesiredTR 右臂期望位姿
     * @param torso_coeff 躯干权重
     * @param iter_out 迭代次数
     * @param posture_scale 回正系数[0,1]，默认1；0关闭回正，1完整回正
     * @return 17×1关节角，单位rad
     */
    Eigen::Matrix<double, 17, 1> inverseUpperlimb(
        const double *q_torso_left_right,
        const Eigen::Matrix4d &DesiredTL,
        const Eigen::Matrix4d &DesiredTR,
        double torso_coeff,
        int &iter_out, double posture_scale = 1.0) const;
    /**
     * @brief 全身逆运动学
     * @ingroup IK
     * @param q_base_torso_left_right 当前底盘-躯干-左臂-右臂关节角
     * @param DesiredTL 左臂期望位姿
     * @param DesiredTR 右臂期望位姿
     * @param torso_coeff 躯干权重
     * @return 19×1关节角，单位rad
     */
    Eigen::Matrix<double, 19, 1> inverseWholebody(
        const double *q_base_torso_left_right,
        const Eigen::Matrix4d &DesiredTL,
        const Eigen::Matrix4d &DesiredTR,
        double torso_coeff) const;

    // ---------------- 运动学工具函数 ----------------
    /**
     * @brief 获取绕X轴旋转矩阵
     * @param angle 绕X轴旋转关节角，单位rad
     * @return 3×3旋转矩阵
     */
    Eigen::Matrix3d rotx(const double angle) const;
    /**
     * @brief 获取绕Y轴旋转矩阵
     * @param angle 绕Y轴旋转关节角，单位rad
     * @return 3×3旋转矩阵
     */
    Eigen::Matrix3d roty(const double angle) const;
    /**
     * @brief 获取绕Z轴旋转矩阵
     * @param angle 绕Z轴旋转关节角，单位rad
     * @return 3×3旋转矩阵
     */
    Eigen::Matrix3d rotz(const double angle) const;
    /**
     * @brief 获取平移变换矩阵
     * @param x,y,z 平移向量，单位m
     * @return 4×4旋转矩阵
     */
    Eigen::Matrix4d transl(const double x, const double y, const double z) const;
    /**
     * @brief 获取偏差，相对基坐标系
     * @ingroup Utility
     * @param Tc 当前齐次矩阵
     * @param Td 期望齐次矩阵
     * @return
     */
    Eigen::Matrix<double, 6, 1> SE3_delta(
        const Eigen::Matrix4d &Tc,
        const Eigen::Matrix4d &Td) const;

    // ---------------- 可操作度梯度 ----------------
    /**
     * @brief 获取左臂可操作度的雅可比矩阵
     * @ingroup Kinematics
     * @param q_left 左臂当前关节角，维度7×1，单位rad
     * @param Jm 左臂可操作度雅可比矩阵
     */
    void manipulabilityGradientLeft(
        const double *q_left,
        Eigen::Matrix<double, 7, 1> &Jm) const;
    /**
     * @brief 获取右臂可操作度的雅可比矩阵
     * @ingroup Kinematics
     * @param q_right 右臂当前关节角，维度7×1，单位rad
     * @param Jm 右臂可操作度雅可比矩阵
     */
    void manipulabilityGradientRight(
        const double *q_right,
        Eigen::Matrix<double, 7, 1> &Jm) const;
    /**
     * @brief 获取左侧支链可操作度的雅可比矩阵
     * @ingroup Kinematics
     * @param q_torso_left 躯干+左臂当前关节角，维度10×1，单位rad
     * @param Jm 左侧支链可操作度雅可比矩阵
     */
    void manipulabilityGradientLeftChain(
        const double *q_torso_left,
        Eigen::Matrix<double, 10, 1> &Jm) const;
    /**
     * @brief 获取右侧可操作度的雅可比矩阵
     * @ingroup Kinematics
     * @param q_torso_right 躯干+右臂当前关节角，维度10×1，单位rad
     * @param Jm 右侧支链可操作度雅可比矩阵
     */
    void manipulabilityGradientRightChain(
        const double *q_torso_right,
        Eigen::Matrix<double, 10, 1> &Jm) const;

    // ---------------- 自碰撞距离 ----------------
    /**
     * @brief 计算自碰撞距离
     * @ingroup Utility
     * @param q 当前上肢关节角，维度17×1，单位rad
     * @param dis_left_right 左右臂距离
     * @param dis_left_T3    左臂距离
     * @param dis_left_T2    左臂距离
     * @param dis_right_T3   右臂距离
     * @param dis_right_T2   右臂距离
     * @return
     */
    void getSelfDistance(
        const Eigen::Matrix<double, 17, 1> &q,
        double &dis_left_right,
        double &dis_left_T3,
        double &dis_left_T2,
        double &dis_right_T3,
        double &dis_right_T2) const;

    // ---------------- ZMP ----------------
    /**
     * @brief 计算上肢ZMP
     * @ingroup Utility
     * @param q 当前上肢关节角，维度17×1，单位rad
     * @param dq 当前上肢角速度，维度17×1，单位rad
     * @param ddq 当前上肢角加速度，维度17×1，单位rad
     * @return ZMP坐标（X,Y）
     */
    Eigen::Matrix<double, 2, 1> getZMP(
        const Eigen::Matrix<double, 17, 1> &q,
        const Eigen::Matrix<double, 17, 1> &dq,
        const Eigen::Matrix<double, 17, 1> &ddq) const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};