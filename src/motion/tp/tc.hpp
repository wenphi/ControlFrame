#pragma once
#include "../motionType.hpp"
#include "../arithmetic/quintic.hpp"
#include "liborocosKdl/include/frames_io.hpp"
#include <stdint.h>
namespace MOTION
{
//轨迹计算的所有类型
enum MoveTypes_t
{
    SINGAL_JOINT = 0, //单轴运动
    P2P,              //点到点运动
    P2P_FASTER,       //快速点到点
    LINE,             //直线
    CIRCLE,           //圆弧
};

//单轴运动
struct MoveSingalJoint_t
{
    bool is_arm;          //是否为机械臂关节
    uint32_t joint_index; //关节号
    double start;         //启动位姿
    double end;           //停止位置
    double u_vec;
};

struct MoveP2PFasterStruct_t
{
    IntePosition_t start; //启动位置
    IntePosition_t end;   //停止位置
    bool p2p_faster_done;
};
//点到点运动结构体
struct MoveP2PStruct_t
{
    IntePosition_t start; //启动位置
    IntePosition_t end;   //停止位置
    IntePosition_t u_vec; //vector
};

//直线运动结构体
struct MoveLineStruct_t
{
    IntePose_t start;
    IntePose_t end;
    IntePose_t u_vec;
};

//圆弧运动结构体
struct MoveCircleStruct_t
{
    IntePose_t start;
    IntePose_t end;
    IntePose_t u_vec; /*u_vec.frame.p运动方向，u_vec.frame.M姿态变量
                            u_vec.ext_joint，外部轴关节变量*/
    KDL::Vector center;
    KDL::Vector normal;
    double r;
    double angle;
    double chord_error;
};

//速度状态机
enum SpeedStateMachine_t
{
    ACCEL_S0 = 0,
    ACCEL_S1,
    ACCEL_S2,
    ACCEL_S3,
    ACCEL_S4,
    ACCEL_S5,
    ACCEL_S6,
    ACCEL_S7
};

//插补类型
enum MoveBlendType_t
{
    NONE_BLEND,
    VEL_BLEND,
    CIRCLE_BLEND,
};

//运动类型
enum MovePlanType_t
{
    PLAN_JOINT,
    PLAN_CART,
};
//插补参数
struct BlendParam_t
{
    UserOvl_t vel_param;
};

//tc运行参数
struct TcRunParam
{
    double limit_vm;  //限制后的vm
    double limit_acc; //限制后的acc
    double limit_ve;  //限制后的ve
};

//轨迹计算结构体
struct TrajectoryCalc
{
    bool valid;                       //tc是否有效
    bool active;                      //该段运动被激活
    MoveTypes_t move_type;            //运动类型
    MoveBlendType_t blend_type;       //过渡插补类型
    BlendParam_t blend_param;         //过渡插补参数
    MoveSingalJoint_t s_joint;        //单轴运动参数
    MoveP2PStruct_t p2p;              //点到点运动参数
    MoveP2PFasterStruct_t p2p_faster; //点到点快速运动
    MoveLineStruct_t line;            //直线运动参数
    MoveCircleStruct_t circle;        //圆弧运动参数
    MovePlanType_t plan_type;         //规划类型
    double cycle_time;                //插补周期
    double target;                    //目标位置
    double req_vm;                    //目标速度
    double req_acca;                  //目标加速度
    double req_accd;                  //目标加速度
    double req_jerk;                  //加加速度
    double req_vs;                    //启动速度
    double req_ve;                    //停止速度
    double curr_vel;                  //当前速度
    double curr_acc;                  //当前加速度
    double progress;                  //progress
    SpeedStateMachine_t accel_state;  //速度控制状态机
    bool tmag_zero;                   //true target为0
    bool tc_done;                     //完成
    uint32_t id;                      //tc的ID号
    double temp_Blend_vel;            //用于暂存过渡段的最大速度
    bool on_final_dec;                //在最后的减速段
    double progress_error;
    int ik_flag;

    //for s_curve
    double turnvel;
    double hightestvel;
    double lowestdist;
    SpeedStateMachine_t save_state;

    //五次多项式
    double toltal_time;
    double target_vel;
    double acc_vel_rate; //加速度和速度比值
    double dec_vel_rate; //减速度和速度比值
    QuinticParam q_param;
    QuinticFactor q_factor;

    //aloha
    double t;     //current 时间
    double t1;    //第一段时间
    double t2;    //第二段时间
    double t3;    //第三段时间
    double S1;    //第一段路程
    double S2;    //第二段路程
    double S3;    //第三段路程
    double Vb;    //变速速度过渡值
    double acc1b; //变速加速度过渡值
    double Sb;    //变速位置过渡值
    double tb;    //变速时间过渡值
    double Sfi;   //初始总路程
    double first; //初始限速决定值

    //for quintic
    double quintic_time;
    QuinticFactor quintic_factor;

    //功能类
    void tcRunCycle(TcRunParam &tc_param); //计算
    bool initialTc(double cycle_time_i,
                   double vs, double ve, double vm,
                   double acca, double accd, double jerk,
                   double target_i); //初始化tc
    void clearTc();                  //急停tc
    void activeTc();

    //private:
    void tcRunCycleT(TcRunParam &tc_param);       //T曲线
    void tcRunCycleS(TcRunParam &tc_param);       //S曲线
    void tcRunCycleQuintic(TcRunParam &tc_param); //5次多项式规划
    void tcRunCycleQuinticS(TcRunParam &tc_param);
    void tcRunCycleQuintic_aloha(TcRunParam &tc_param);
};
} // namespace MOTION