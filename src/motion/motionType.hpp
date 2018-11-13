#pragma once
#include "../base/stateBase.hpp"
#include "liborocosKdl/include/frames_io.hpp"
#define MAX_ARM_JOINT_NUM 7 //最大的关节数量
#define MAX_EXT_JOINT_NUM 3 //最大的扩展轴数量

namespace MOTION
{
enum motionCmd_t
{
    MOTION_CMD_UNDEFINE = msgCmd_t::MSG_CMD_MOTION_UNDEFINE,
    MOTION_CMD_ADD_LINE
};
//内部的位置
struct IntePosition_t
{
    double arm_joint[MAX_ARM_JOINT_NUM];
    double ext_joint[MAX_EXT_JOINT_NUM];
};

//内部姿态
struct IntePose_t
{
    KDL::Frame frame;
    double ext_joint[MAX_EXT_JOINT_NUM];
};

//位姿结构体
struct Pose_t
{
    Pose_t() {}
    Pose_t(uint32_t ext_joint_num)
        : ext_joint(ext_joint_num)
    {
    }
    KDL::Frame frame;        //机械臂位姿
    KDL::JntArray ext_joint; //扩展轴位置
};

//位置结构体
struct Position_t
{
    Position_t() {}
    Position_t(uint32_t arm_joint_num, uint32_t ext_joint_num)
        : arm_joint(arm_joint_num),
          ext_joint(ext_joint_num)
    {
    }
    KDL::JntArray arm_joint; //关节位置
    KDL::JntArray ext_joint; //扩展轴位置
};

//机械臂的笛卡尔空间
struct RobotCartSpace_t
{
    KDL::Vector min_point;
    KDL::Vector max_point;
};

//用户速度base
struct UserVelBase_t
{
    double vel;  //速度
    double acca; //加速度
    double accd; //减速度
    double jerk; //加加速度
};
//用户速度
struct UserVel_t
{
    UserVelBase_t jv; //关节速度
    UserVelBase_t lv; //线性路径速度
    UserVelBase_t rv; //旋转速度
};

//用户逼近参数
struct UserOvl_t
{
    double rel; //相对逼近参数
    double abs; //绝对逼近参数
    double rad; //旋转逼近参数
};

//机械臂配置结构体
struct MotionConfig_t
{
    MotionConfig_t()
    {
        robot_name = "";
        arm_joint_num = 0;
        ext_joint_num = 0;
    }
    std::string robot_name; //机械臂名称
    // RobotModel_t robot_model; //机械臂模型
    // STRUCT_DH dh[7];          //D-H参数

    //机械臂关节参数
    uint32_t arm_joint_num;               //机械臂的关节数量
    KDL::JntArray arm_joint_abs_limit_h;  //关节绝对位置限制,最大值
    KDL::JntArray arm_joint_abs_limit_l;  //关节绝对位置限制,最小值
    KDL::JntArray arm_joint_soft_limit_h; //关节软件位置限制,最大值
    KDL::JntArray arm_joint_soft_limit_l; //关节软件位置限制,最小值
    KDL::JntArray arm_joint_max_vel;
    KDL::JntArray arm_joint_max_acc;
    KDL::JntArray arm_joint_max_dec;
    KDL::JntArray arm_joint_max_jerk;
    KDL::JntArray arm_joint_limit_vel;
    KDL::JntArray arm_joint_limit_acc;
    KDL::JntArray arm_joint_limit_dec;

    //扩展轴参数
    uint32_t ext_joint_num;               //扩展关节数
    KDL::JntArray ext_joint_abs_limit_h;  //关节绝对位置限制,最大值
    KDL::JntArray ext_joint_abs_limit_l;  //关节绝对位置限制,最小值
    KDL::JntArray ext_joint_soft_limit_h; //关节软件位置限制,最大值
    KDL::JntArray ext_joint_soft_limit_l; //关节软件位置限制,最小值
    KDL::JntArray ext_joint_max_vel;
    KDL::JntArray ext_joint_max_acc;
    KDL::JntArray ext_joint_max_dec;
    KDL::JntArray ext_joint_max_jerk;
    KDL::JntArray ext_joint_limit_vel;
    KDL::JntArray ext_joint_limit_acc;
    KDL::JntArray ext_joint_limit_dec;

    //笛卡尔参数
    double arm_cart_max_vel;
    double arm_cart_max_acc;
    double arm_cart_max_dec;
    double arm_cart_max_jerk;

    //末端夹具描述
    KDL::Frame end_effector;

    //方法
    void resizeArmJoint(uint32_t arm_joint_num_i)
    {
        arm_joint_abs_limit_h.resize(arm_joint_num_i);
        arm_joint_abs_limit_l.resize(arm_joint_num_i);
        arm_joint_soft_limit_h.resize(arm_joint_num_i);
        arm_joint_soft_limit_l.resize(arm_joint_num_i);
        arm_joint_max_vel.resize(arm_joint_num_i);
        arm_joint_max_acc.resize(arm_joint_num_i);
        arm_joint_max_dec.resize(arm_joint_num_i);
        arm_joint_max_jerk.resize(arm_joint_num_i);
        arm_joint_limit_vel.resize(arm_joint_num_i);
        arm_joint_limit_acc.resize(arm_joint_num_i);
        arm_joint_limit_dec.resize(arm_joint_num_i);
    }
    void resizeExtJoint(uint32_t ext_joint_num_i)
    {
        ext_joint_abs_limit_h.resize(ext_joint_num_i);
        ext_joint_abs_limit_l.resize(ext_joint_num_i);
        ext_joint_soft_limit_h.resize(ext_joint_num_i);
        ext_joint_soft_limit_l.resize(ext_joint_num_i);
        ext_joint_max_vel.resize(ext_joint_num_i);
        ext_joint_max_acc.resize(ext_joint_num_i);
        ext_joint_max_dec.resize(ext_joint_num_i);
        ext_joint_max_jerk.resize(ext_joint_num_i);
        ext_joint_limit_vel.resize(ext_joint_num_i);
        ext_joint_limit_acc.resize(ext_joint_num_i);
        ext_joint_limit_dec.resize(ext_joint_num_i);
    }
};

//motion的错误码
enum MotionErrorCode_t
{
    //DEBUG
    MOTION_DEBUG_UNDEFINE = 15000, //未定义
    MOTION_DEBUG_CONFIG,           //配置debug
    MOTION_DEBUG_CMD,              //指令debug
    MOTION_DEBUG_CHECK,            //命令检测debug
    MOTION_DEBUG_MOTOR,            //电机debug
    MOTION_DEBUG_DEVICE,           //设备状态debug
    MOTION_DEBUG_KDL,              //正逆解debug
    MOTION_DEBUG_SAFE,             //限位debug
    MOTION_DEBUG_TP,               //轨迹规划debug
    MOTION_DEBUG_QUERY,            //查询debug
    MOTION_DEBUG_HOOK,             //线程hook_debug
    //INFO
    MOTION_INFO_UNDEFINE = 15200,     //未定义
    MOTION_INFO_DEVICE_POWERON,       //继电器上电
    MOTION_INFO_DEVICE_POWEROFF,      //继电器断电
    MOTION_INFO_DEVICE_IDLE,          //设备处于空闲状态
    MOTION_INFO_DEVICE_DRAG,          //设备处于拖拽状态
    MOTION_INFO_DEVICE_TEACHING,      //设备处于示教状态
    MOTION_INFO_DEVICE_EXIT_TEACHING, //设备退出示教状态
    MOTION_INFO_DEVICE_SCRIPT,        //设备处于脚本运行状态
    MOTION_INFO_DEVICE_EXIT_SCRIPT,   //设备退出脚本运行状态
    MOTION_INFO_TP_START,             //开始轨迹插补
    MOTION_INFO_TP_DONE,              //结束轨迹插补
    MOTION_INFO_DEVICE_HOMED,         //设备回零完成
    MOTION_INFO_CONFIG_SUCCEED,       //配置文件加载完成
    MOTION_INFO_CMD_DONE,             //指令写入成功
    MOTION_INFO_DEVICE_EXIT_DRAG,     //未定义设备退出拖拽
    MOTION_INFO_CONFIG_REPORT,        //motion配置信息显示
    MOTION_INFO_DESTROY,              //创建的类已销毁
    MOTION_INFO_CMD_WRITE,            //开始写入指令
    MOTION_INFO_KDL_SET_EFFECTOR,     //设置夹具
    MOTION_INFO_KDL_REMOVE_EFFECTOR,  //移除夹具
    //WARNING
    MOTION_WARNING_UNDEFINE = 15400,         //未定义
    MOTION_WARNING_CMD_OVERTIME,             //指令写入超时
    MOTION_WARNING_CMD_BUZY,                 //指令写入繁忙
    MOTION_WARNING_CMD_QUEUE_FULL,           //添加运动-运动队列已满
    MOTION_WARNING_CHECK_OVER_SOFT_LIMIT,    //添加运动-运动将超出软限位
    MOTION_WARNING_DEVICE_EMERGENCY,         //设备处于急停状态
    MOTION_WARNING_DEVICE_EMERGENCY_RELEASE, //设备解除急停
    MOTION_WARNING_DEVICE_RUNNING,           //设备即将开始运动
    //ERROR
    MOTION_ERROR_UNDEFINE = 15600,            //未定义
    MOTION_ERROR_CHECK_JOINT_INDEX_ERROR,     //指令错误-关节索引超出范围
    MOTION_ERROR_CHECK_UNCONFIG,              //指令错误-motion未成功配置
    MOTION_ERROR_CHECK_OVER_ABS_LIMIT,        //指令错误-运动将超出硬限位
    MOTION_ERROR_CHECK_STATE_EMERGENCY,       //指令错误-设备处于急停状态
    MOTION_ERROR_CHECK_STATE_NOT_IDLE,        //指令错误-设备不处于空闲状态
    MOTION_ERROR_SAFE_OVER_ABS_LIMIT,         //运动超出硬限位
    MOTION_ERROR_SAFE_OVER_SOFT_LIMIT,        //运动超出软限位
    MOTION_ERROR_KDL_FORWARD_KINEMATIC_ERROR, //正解错误
    MOTION_ERROR_KDL_INVERSE_KINEMATIC_ERROR, //逆解错误
    MOTION_ERROR_KDL_JOINT_SINGULAR,          //关节奇异
    MOTION_ERROR_KDL_ARM_JOINT_NUM_ERROR,     //内部关节轴数量错误
    MOTION_ERROR_KDL_EXT_JOINT_NUM_ERROR,     //外部扩展轴数量错误
    MOTION_ERROR_CMD_ADD_MOVE_ERROR,          //添加指令失败
    MOTION_ERROR_CMD_DRAG_CURRENT,            //指令错误-拖拽电流超出限制范围
    MOTION_ERROR_CHECK_NOT_HOMED,             //指令错误-关节未回零完成
    MOTION_ERROR_TP_ERROR_RUN_MODE,           //轨迹插补模式错误
    MOTION_ERROR_TP_KINEMATIC_FLAG,           //轨迹插补逆解标志不匹配
    MOTION_ERROR_QUERY_ERROR,                 //查询错误
    MOTION_ERROR_CHECK_USER_VEL_ERROR,        //指令错误-运动参数配置错误,不在限定范围
    MOTION_ERROR_CHECK_USER_OVL_ERROR,        //指令错误-逼近参数配置错误,不在限定范围
    MOTION_ERROR_CHECK_FEED_RATE_ERROR,       //指令错误-速度进给率设置错误,不在限定范围
    MOTION_ERROR_MOTOR_BEEN_REGISTED,         //电机已被绑定
    MOTION_ERROR_MOTOR_START_SERCH_ERROR,     //电机开始回零失败
    //FATAL
    MOTION_FATAL_FATAL_UNDEFINE = 15800, //未定义
    MOTION_FATAL_CONFIG_OPEN_FAILED,     //配置文件打开失败
    MOTION_FATAL_CONFIG_FORMAT_ERROR,    //配置文件格式错误
    MOTION_FATAL_CONFIG_PARSE_ERROR,     //配置文件解析错误
    MOTION_FATAL_CONFIG_FAILED,          //motion配置失败
    MOTION_FATAL_MOTOR_UNREGISTE,        //电机未绑定
    MOTION_FATAL_MOTOR_INSIDE_ERROR,     //电机内部错误
    MOTION_FATAL_MOTOR_UNCONNECT,        //电机未链接
    MOTION_FATAL_HOOK_CONFIG_FAILED,     //motion线程启动配置失败
};

//关节状态
enum JointStatus_t
{
    UNINITIAL = 0,
    JOINT_RUNING,  //关节运动中
    JOINT_WARNING, //出现警告
    JOINT_ERROR,   //关节错误
};
} // namespace MOTION
