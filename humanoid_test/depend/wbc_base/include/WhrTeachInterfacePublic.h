#pragma once
#include <memory>
#include <atomic>
#include "WhrModelPublic.h"
#include "HYYRobotInterface.h"
using namespace HYYRobotBase;
/**
 * @brief 机器人上肢当前关节状态
 *
 * 关节排列顺序：
 * q_upper / dq_upper =
 * [躯干3自由度, 左臂7自由度, 右臂7自由度]
 */
struct RobotState
{
    Eigen::Matrix<double, 3, 1> q_torso = Eigen::Matrix<double, 3, 1>::Zero();
    Eigen::Matrix<double, 7, 1> q_left = Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 7, 1> q_right = Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 10, 1> q_torso_left = Eigen::Matrix<double, 10, 1>::Zero();
    Eigen::Matrix<double, 10, 1> q_torso_right = Eigen::Matrix<double, 10, 1>::Zero();
    Eigen::Matrix<double, 17, 1> q_upper = Eigen::Matrix<double, 17, 1>::Zero();
    Eigen::Matrix<double, 3, 1> dq_torso = Eigen::Matrix<double, 3, 1>::Zero();
    Eigen::Matrix<double, 7, 1> dq_left = Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 7, 1> dq_right = Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 10, 1> dq_torso_left = Eigen::Matrix<double, 10, 1>::Zero();
    Eigen::Matrix<double, 10, 1> dq_torso_right = Eigen::Matrix<double, 10, 1>::Zero();
    Eigen::Matrix<double, 17, 1> dq_upper = Eigen::Matrix<double, 17, 1>::Zero();
};

class WhrTeachInterface
{
public:
    explicit WhrTeachInterface(WhrModel &model);
    ~WhrTeachInterface();

    WhrTeachInterface(const WhrTeachInterface &) = delete;
    WhrTeachInterface &operator=(const WhrTeachInterface &) = delete;

    /** @brief 机器人当前状态，调用 GetRobotState() 后更新 */
    RobotState robot_state;

    /**
     * @brief 读取并刷新机器人当前关节位置和关节速度
     * @return 0：成功；其它：失败
     */
    int GetRobotState();

    /**
     * @brief 请求开启笛卡尔空间jog
     * @param robot_index 机器人索引，0:左臂，1:右臂
     * @param axis_index 笛卡尔轴索引，0:x，1:y，2:z，3:rx，4:ry，5:rz
     * @param dir 运动方向，>0:正方向，<0:负方向，=0:无效
     * @param speed_scale 运动速度，0-1
     * @return 0:执行成功；负数:参数错误；正数:已有jog正在运行
     */
    int cartesian_jog_interface(int robot_index, int axis_index, int dir, double speed_scale);

    /**
     * @brief 请求停止笛卡尔空间jog
     * @return 0:执行成功
     */
    int stop_cartesian_jog_interface();
    /**
     * @brief 开启单臂拖动示教
     * @param arm_mode 0：左臂，1：右臂，2：双臂
     * @param feel_mode 0：轻盈，1：稳健
     * @param drag_mode 0：自由，1：仅平移，2：仅旋转
     */
    int start_drag_teach_interface(int arm_mode, int feel_mode, int drag_mode);
    /**
     * @brief 关闭当前拖动示教
     */
    int stop_drag_teach_interface();
    /**
     * @brief 获取拖动示教当前状态
     * @param arm_mode 拖动对象，0：左臂，1：右臂，2：双臂
     * @param feel_mode 拖动手感，0：轻盈，1：稳健
     * @param drag_mode 拖动模式，0：自由，1：仅平移，2：仅旋转
     * @param error_code 当前或最近一次拖动控制错误码
     * @return 0：未运行，1：运行中
     */
    int get_drag_teach_state(int &arm_mode, int &feel_mode, int &drag_mode, int &error_code) const;
    /**
     * @brief 获取双臂关节观测外力
     * @param left_force  左臂关节观测外力
     * @param right_force 右臂关节观测外力
     * @return 0：获取成功；-1：输入指针为空
     */
    int get_observer_force(double left_force[7], double right_force[7]) const;
    /**
     * @brief 启动物理按钮拖动示教监测
     * @param feel_mode 0：轻盈，1：稳健
     * @param drag_mode 0：自由，1：仅平移，2：仅旋转
     * @return 0：启动成功；1：已经运行；负数：启动失败
     */
    int start_physical_drag_button_interface(int feel_mode, int drag_mode);
    /**
     * @brief 关闭物理按钮拖动示教
     * @return 0：停止成功
     */
    int stop_physical_drag_button_interface();
    /**
     * @brief 阻抗控制
     * @param robot_index 机器人索引，0:左臂，1:右臂
     * @param K 关节刚度
     * @param D 关节阻尼
     * @return 0: 执行成功;其它：失败
     */
    int ImpedanceControl(int robot_index, const double *K, const double *D);
    /**
     * @brief 停止当前关节阻抗控制
     * @return 0：正常停止；其他：底层控制错误码
     */
    int StopImpedanceControl();
    /**
     * @brief 获取关节阻抗控制状态
     * @param robot_index 当前或最近一次运行的机械臂索引
     * @param error_code 当前或最近一次控制错误码
     * @return 0：未运行，1：运行中
     */
    int GetImpedanceControlState(int &robot_index, int &error_code) const;
    /**
     * @brief 上肢联动线性指令
     * @param DesiredTL 左臂目标末端位姿（4x4齐次矩阵，位置+姿态）
     * @param DesiredTR 右臂目标末端位姿（4x4齐次矩阵，位置+姿态）
     * @param plan_time 规划时间，单位:s
     * @param torso_coeff 躯干参与权重系数, 必须大于0；小于1时增强躯干参与，等于1时躯干与双臂同权重，大于1时抑制躯干参与
     * @return 0: 执行成功;其它：失败
     */
    int UpperMoveL(Eigen::Matrix4d &DesiredTL, Eigen::Matrix4d &DesiredTR, double plan_time, double torso_coeff);
    /**
     * @brief 上肢关节联动指令
     * @param DesiredTL 左臂目标末端位姿（4x4齐次矩阵，位置+姿态）
     * @param DesiredTR 右臂目标末端位姿（4x4齐次矩阵，位置+姿态）
     * @param rspeed 转动速度，单位s
     * @param torso_coeff 躯干参与权重系数, 必须大于0；小于1时增强躯干参与，等于1时躯干与双臂同权重，大于1时抑制躯干参与
     * @return 0: 执行成功;其它：失败
     */
    int UpperMoveJ(Eigen::Matrix4d &DesiredTL, Eigen::Matrix4d &DesiredTR, HYYRobotBase::speed *rspeed, double torso_coeff);
    /**
     * @brief 上肢笛卡尔伺服指令
     * @param DesiredTL 左臂目标末端位姿（4x4齐次矩阵，位置+姿态）
     * @param DesiredTR 右臂目标末端位姿（4x4齐次矩阵，位置+姿态）
     * @param torso_coeff 躯干参与权重系数（0~1）: 接近0，躯干更容易参与；接近1，躯干参与度与手臂一样
     * @param time 数据时间
     * @param posture_scale 回正系数[0,1]，默认1；0关闭回正，1完整回正
     * @return 0: 执行成功并完成MoveL轨迹;其它：失败
     */
    int UpperServoCartesian(Eigen::Matrix4d &DesiredTL, Eigen::Matrix4d &DesiredTR, double torso_coeff, double time, double posture_scale = 1.0);
    /**
     * @brief 位姿位姿向量转齐次矩阵
     * @param T 齐次矩阵
     * @param pose 控制器位姿数据
     * @return
     */
    void RobposeToHomogeneousMatrix(Eigen::Matrix4d &T, robpose *pose);
    /**
     * @brief 齐次矩阵转位姿向量
     * @param T 齐次矩阵
     * @param pose 控制器位姿数据
     * @return
     */
    void HomogeneousMatrixTorobpose(const Eigen::Matrix4d &T, robpose *pose);
    /**
     * @brief 运行关节外力矩观测器
     * @param robot_index 机械臂索引，0：左臂，1：右臂
     * @param K0 观测器增益
     * @param stop_requested 外部停止标志
     * @return 0：正常结束；其他：错误码
     */
    int ExternalTorqueObserver(int robot_index, double K0, const std::atomic<bool> &stop_requested);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};