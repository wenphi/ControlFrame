#include "tc.hpp"
using namespace MOTION;
using namespace KDL;
#define Bl_Square(x) x *x

inline double blSaturate(double x, double max)
{
    if (x > max)
        return max;
    else if (x < -max)
        return -max;
    else
        return x;
}

void TrajectoryCalc::tcRunCycle(TcRunParam &tc_param)
{
    if (active)
    {
        // tcRunCycleT(tc_param);
        // tcRunCycleS(tc_param);
        tcRunCycleQuinticS(tc_param);
        // tcRunCycleQuintic_aloha(tc_param);
    }
    return;
}

bool TrajectoryCalc::initialTc(double cycle_time_i,
                               double vs, double ve, double vm,
                               double acca, double accd, double jerk,
                               double target_i)
{
    cycle_time = cycle_time_i;
    req_vs = vs;
    req_ve = ve;
    req_vm = vm;
    req_acca = acca;
    req_accd = accd;
    req_jerk = jerk;
    target = target_i;
    progress = 0;
    curr_vel = req_vs;
    curr_acc = 0;
    active = false;
    tc_done = false;
    valid = true;
    on_final_dec = false;
    progress_error = 0;

    //S-curve
    accel_state = ACCEL_S3;

    //quintic
    quintic_time = 0.0;
    QuinticParam q_param;
    q_param.p0 = 0;
    q_param.p1 = target;
    q_param.dp0 = q_param.ddp0 = q_param.dp1 = q_param.ddp1 = 0.0;
    quinticComputeFactor(q_param, quintic_factor, 2.0);

    //aloha
    t = 0;
    Vb = 0;
    acc1b = 0;
    Sb = 0;
    tb = 0;
    Sfi = target;
    first = false;
    S1 = 0;
    S2 = 0;
    S3 = 0;
    t1 = 0;
    t2 = 0;
    t3 = 0;

    return true;
}

void TrajectoryCalc::clearTc()
{
    active = false;
    valid = false;
}

void TrajectoryCalc::activeTc()
{
    active = true;
}

void TrajectoryCalc::tcRunCycleT(TcRunParam &tc_param)
{
    double dx = target - progress;
    double maxaccel = tc_param.limit_acc;
    double discr_term1 = Bl_Square(tc_param.limit_ve);
    double discr_term2 = maxaccel * (2.0 * dx - curr_vel * cycle_time);
    double tmp_adt = maxaccel * cycle_time * 0.5;
    double discr_term3 = Bl_Square(tmp_adt);
    double discr = discr_term1 + discr_term2 + discr_term3;
    double maxnewvel = -tmp_adt;
    if (discr > discr_term3)
    {
        maxnewvel += sqrt(discr);
    }
    double newvel = blSaturate(maxnewvel, tc_param.limit_vm);
    double maxnewaccel = (newvel - curr_vel) / cycle_time;
    curr_acc = blSaturate(maxnewaccel, maxaccel);
    double v_next = curr_vel + curr_acc * cycle_time;
    if (v_next < 0)
    {
        v_next = 0;
        if ((target - progress) < (curr_vel * cycle_time))
        {
            progress = target;
            tc_done = true;
        }
    }
    else
    {
        double displacement = (v_next + curr_vel) * 0.5 * cycle_time;
        progress += displacement;
        if ((target - progress) < 1e-8)
        {
            progress = target;
            tc_done = true;
        }
    }
    curr_vel = v_next;
    if (curr_acc < 0)
    {
        on_final_dec = true;
    }
    else
    {
        on_final_dec = false;
    }
    return;
}

void TrajectoryCalc::tcRunCycleS(TcRunParam &tc_param)
{

    double t = 0, t1 = 0, vel = 0, tempvel = 0, dist = 0;
    double next_progress, next_vel, next_accel;
    if (false != target)
    {
        switch (accel_state)
        {
        case ACCEL_S0:

            progress = progress + curr_vel * cycle_time + 0.5 * curr_acc * cycle_time * cycle_time + 1.0 / 6.0 * req_jerk * cycle_time * cycle_time * cycle_time;
            curr_vel = curr_vel + curr_acc * cycle_time + 0.5 * req_jerk * cycle_time * cycle_time;
            curr_acc = curr_acc + req_jerk * cycle_time;
            //next_cycle
            next_progress = progress + curr_vel * cycle_time + 0.5 * curr_acc * cycle_time * cycle_time + 1.0 / 6.0 * req_jerk * cycle_time * cycle_time * cycle_time;
            next_vel = curr_vel + curr_acc * cycle_time + 0.5 * req_jerk * cycle_time * cycle_time;
            next_accel = curr_acc + req_jerk * cycle_time;
            //1.检查加速度是否达到最大值
            if (curr_acc - tc_param.limit_acc > 1e-6)
            {
                // curr_acc = maxaccel;
                accel_state = ACCEL_S1;
                break;
            }
            //2.检查速度是否达到约束值
            vel = (tc_param.limit_vm + hightestvel) / 2;
            if (next_vel - vel > 1e-6)
            {
                accel_state = ACCEL_S2;
                break;
            }
            //3.检查位移是否达到约束值
            //3.1前瞻s0，s2，s4,s6 给定位移使得速度无法达到最大情况
            if (curr_vel >= tc_param.limit_ve)
            {
                tempvel = next_vel - hightestvel;
                // t=sqrt(2*req_jerk*tempvel)/req_jerk;   //当前加速所用的时间
                t = floor((next_accel / req_jerk) / cycle_time);
                t = t * cycle_time;
                dist = t * (next_vel) + (req_jerk * t * t * t) / 3; //加速段的路程
                tempvel = hightestvel + 2 * tempvel;                //当前进入s2的峰顶速度
                tempvel = tempvel - tc_param.limit_ve;              //减速段的速度变化量
                t = floor((req_accd / req_jerk) / cycle_time);
                t = t * cycle_time;
                vel = (req_jerk * t * t) / 2;
                if (tempvel / 2 > vel)
                {
                    t = (req_accd / req_jerk);
                    t1 = ((tempvel - 2 * vel) / req_accd);
                    dist = dist + (2 * t + t1) * ((tempvel / 2) + tc_param.limit_ve) + (tempvel + tc_param.limit_ve) * cycle_time; //加速阶段加上减速阶段，再加一个周期的s3,减去当前加速段走的路程；
                }
                else
                {
                    vel = tempvel / 2;
                    t = sqrt(2 * req_jerk * vel) / req_jerk;
                    dist = dist + 2 * t * (vel + tc_param.limit_ve) + (tempvel + tc_param.limit_ve) * cycle_time; //加速阶段加上减速阶段，再加上一个周期的s3，减去当前加速段走过的路程；
                }
                if (target - next_progress - dist < -1e-6)
                {
                    accel_state = ACCEL_S2;
                    break;
                }
            }
            //3.2 前瞻s0.s2
            else if (curr_vel < tc_param.limit_ve)
            {
                t = next_accel / req_jerk;
                dist = next_vel * t + (req_jerk * t * t * t) / 3; //直接加速到终点所需的位移
                if (target - next_progress - dist < -1e-6)
                {
                    accel_state = ACCEL_S2;
                    break;
                }
            }
            break;

        case ACCEL_S1:

            progress = progress + curr_vel * cycle_time + 0.5 * curr_acc * cycle_time * cycle_time;
            curr_vel = curr_vel + curr_acc * cycle_time;
            //next_cycle
            next_progress = progress + curr_vel * cycle_time + 0.5 * curr_acc * cycle_time * cycle_time;
            next_vel = curr_vel + curr_acc * cycle_time;
            //1.检查速度是否达到约束值
            vel = (curr_acc * curr_acc) / (2 * req_jerk);
            tempvel = tc_param.limit_vm - vel;
            if (curr_vel - tempvel > 1e-6)
            {
                accel_state = ACCEL_S2;
                break;
            }
            //2.判断位移是否达到约束值
            //2.1 终点速度小于当前速度
            if (curr_vel >= tc_param.limit_ve)
            {
                t = floor((curr_acc / req_jerk) / cycle_time);
                t = t * cycle_time;
                vel = (req_jerk * t * t) / 2;
                dist = next_vel * t + ((req_jerk * t * t * t) / 3);
                tempvel = next_vel + vel;
                tempvel = tempvel - tc_param.limit_ve;
                t = floor((req_accd / req_jerk) / cycle_time);
                t = t * cycle_time;
                vel = (req_jerk * t * t) / 2;
                if (tempvel / 2 > vel)
                {
                    t = req_accd / req_jerk;
                    t1 = (tempvel - 2 * vel) / req_accd;
                    dist = dist + (2 * t + t1) * (tc_param.limit_ve + tempvel / 2) + (tempvel + tc_param.limit_ve) * cycle_time; //为消除断尾现象*2
                }
                else
                {
                    t = sqrt(2 * req_jerk * (tempvel / 2)) / req_jerk;
                    dist = dist + 2 * t * (tempvel / 2 + tc_param.limit_ve) + (tempvel + tc_param.limit_ve) * cycle_time; //为消除断尾现象*2
                }
                if (target - next_progress - dist < 0)
                {
                    accel_state = ACCEL_S2;
                    break;
                }
            }
            //2.2终点速度大于当前速度
            else if (curr_vel < tc_param.limit_ve)
            {
                t = floor((curr_acc / req_jerk) / cycle_time);
                t = t * cycle_time;
                dist = next_vel * t + (req_jerk * t * t * t) / 3;
                if (target - next_progress - dist < -1e-6)
                {
                    accel_state = ACCEL_S2;
                    break;
                }
            }
            break;

        case ACCEL_S2:

            progress = progress + curr_vel * cycle_time + 0.5 * curr_acc * cycle_time * cycle_time - 1.0 / 6.0 * req_jerk * cycle_time * cycle_time * cycle_time;
            curr_vel = curr_vel + curr_acc * cycle_time - 0.5 * req_jerk * cycle_time * cycle_time;
            curr_acc = curr_acc - req_jerk * cycle_time;
            //next_cycle
            next_progress = progress + curr_vel * cycle_time + 0.5 * curr_acc * cycle_time * cycle_time - 1.0 / 6.0 * req_jerk * cycle_time * cycle_time * cycle_time;
            next_vel = curr_vel + curr_acc * cycle_time - 0.5 * req_jerk * cycle_time * cycle_time;
            next_accel = curr_acc - req_jerk * cycle_time;
            //1.检查加速度是否达到0；
            if (curr_acc < -1e-6)
            {
                curr_acc = 0;
                accel_state = ACCEL_S3;
                break;
            }
            break;

        case ACCEL_S3:
            //初始阶段来到s3
            //正常加速来到s3
            //s6结束加速度为0来到s3
            curr_acc = 0;
            progress = progress + curr_vel * cycle_time;
            //next_cycle
            next_progress = progress + curr_vel * cycle_time;
            //1.判断剩余的路程是否达到约束条件
            if (curr_vel > tc_param.limit_ve)
            {
                t = floor((req_accd / req_jerk) / cycle_time);
                t = t * cycle_time;
                vel = (req_jerk * t * t) / 2;
                tempvel = curr_vel - tc_param.limit_ve; //减速变化量
                if (tempvel / 2 > vel)
                {
                    t1 = ceil(((tempvel - vel * 2) / req_accd) / cycle_time);
                    t1 = t1 * cycle_time;
                    dist = (2 * t + t1) * (tc_param.limit_ve + tempvel / 2);
                    //理论末速度
                    vel = curr_vel - req_jerk * t * t - t1 * req_accd;
                }
                else
                {
                    vel = tempvel / 2;                       //加速度转折点的速度
                    t = sqrt(2 * req_jerk * vel) / req_jerk; //走一半所需要的时间
                    t = ceil(t / cycle_time);
                    t = t * cycle_time;
                    dist = 2 * t * (tc_param.limit_ve + vel);
                    //理论末速度
                    vel = curr_vel - req_jerk * t * t;
                }
                //1.1满足减速条件
                if (target - progress - dist < 1e-6)
                {
                    //1.2.平台插补设置
                    // tempvel = (vel + curr_vel) / 2;
                    // dist = tempvel * (2 * t + t1);
                    // dist = target - progress - dist;
                    // //使得减速度段有多无少
                    // if (dist >= 1e-6)
                    // {
                    //     turnvel = dist / cycle_time + tc_param.limit_ve;
                    // }
                    hightestvel = curr_vel;
                    lowestdist = progress;
                    accel_state = ACCEL_S4;
                    break;
                }
            }
            //2.位移不满足约束条件时，检查当前速度和最大速度比较
            if (target - next_progress - 2 * curr_vel * cycle_time > dist)
            {
                if ((curr_vel + tc_param.limit_acc * cycle_time) < tc_param.limit_vm)
                {
                    hightestvel = curr_vel;
                    lowestdist = progress;
                    accel_state = ACCEL_S0;
                    break;
                }
                // else if ((curr_vel - tc_param.limit_acc * cycle_time) > tc_param.limit_vm)
                else if ((curr_vel - 1e-6) > tc_param.limit_vm)
                {
                    hightestvel = curr_vel;
                    lowestdist = progress;
                    accel_state = ACCEL_S4;
                    break;
                }
            }
            break;

        case ACCEL_S4:

            progress = progress + curr_vel * cycle_time + 0.5 * curr_acc * cycle_time * cycle_time - 1.0 / 6.0 * req_jerk * cycle_time * cycle_time * cycle_time;
            curr_vel = curr_vel + curr_acc * cycle_time - 0.5 * req_jerk * cycle_time * cycle_time;
            curr_acc = curr_acc - req_jerk * cycle_time;
            //next_cycle
            next_progress = progress + curr_vel * cycle_time + 0.5 * curr_acc * cycle_time * cycle_time - 1.0 / 6.0 * req_jerk * cycle_time * cycle_time * cycle_time;
            next_vel = curr_vel + curr_acc * cycle_time - 0.5 * req_jerk * cycle_time * cycle_time;
            next_accel = curr_acc - req_jerk * cycle_time;
            //1.检查加速度是否达到最小值
            if (next_accel + tc_param.limit_acc < -1e-6)
            {
                accel_state = ACCEL_S5;
                break;
            }
            //2.检查速度是否满足约束条件
            if (curr_vel - tc_param.limit_acc * cycle_time > tc_param.limit_vm)
                vel = (hightestvel + tc_param.limit_vm) / 2;
            else if (curr_vel + tc_param.limit_acc * cycle_time < tc_param.limit_vm)
                vel = (hightestvel + tc_param.limit_ve) / 2;
            if (next_vel - vel < 1e-6)
            {
                accel_state = ACCEL_S6;
                break;
            }
            //3.检查剩余检查剩余路程是否足够进行s6，发生终点速度达不到理论的情况
            //todo：：出现路程小初速度高的情况
            // t=sqrt(2*req_jerk*(hightestvel-curr_vel))/req_jerk;
            // t=floor(t/cycle_time);
            // t=t*cycle_time;
            // dist=curr_vel*2*t;
            // if(tc_target<=dist)
            // {
            //     accel_state=ACCEL_S6;
            //     break;
            // }
            //4.平台插补
            if (curr_vel < tc_param.limit_vm)
            {
                if (curr_vel <= turnvel)
                {
                    save_state = accel_state;
                    accel_state = ACCEL_S7;
                    break;
                }
            }
            //检查s4区间是否出现最大速度上调情况

            break;

        case ACCEL_S5:

            progress = progress + curr_vel * cycle_time + 0.5 * curr_acc * cycle_time * cycle_time;
            curr_vel = curr_vel + curr_acc * cycle_time;
            //next_cycle
            next_progress = progress + curr_vel * cycle_time + 0.5 * curr_acc * cycle_time * cycle_time;
            next_vel = curr_vel + curr_acc * cycle_time;
            //1.检查速度是否达到约束值
            t = floor(((tc_param.limit_acc) / req_jerk) / cycle_time);
            t = t * cycle_time;
            vel = (req_jerk * t * t) / 2;
            if (curr_vel - tc_param.limit_acc * cycle_time >= tc_param.limit_vm)
                tempvel = tc_param.limit_vm + vel;
            else if (curr_vel + tc_param.limit_acc * cycle_time <= tc_param.limit_vm)
                tempvel = tc_param.limit_ve + vel;

            if (next_vel - tempvel < -1e-6)
            {
                accel_state = ACCEL_S6;
                break;
            }
            //2.检查剩余路程是否足够进行s6，发生终点速度达不到理论的情况
            // tempvel=hightestvel-end_vel;
            // if(tempvel/2>vel)
            // {
            //     t1=(tempvel-2*vel)/maxaccel;
            //     dist=(end_vel+tempvel/2)*(t1+2*t);
            // }
            // else
            // {
            //     dist=t*(tempvel+end_vel);
            // }
            // if(target<dist)
            // {
            //     tempvel=curr_vel-vel;
            //     dist=tempvel*t+(req_jerk*t*t*t)/6;
            //     if(tc_target-progress<=dist)
            //     {
            //         accel_state=ACCEL_S6;
            //         break;
            //     }
            // }
            //3.进行平台插补修正
            if (curr_vel < tc_param.limit_vm)
            {
                if (curr_vel <= turnvel)
                {
                    save_state = accel_state;
                    accel_state = ACCEL_S7;
                    break;
                }
            }
            break;

        case ACCEL_S6:

            progress = progress + curr_vel * cycle_time + 0.5 * curr_acc * cycle_time * cycle_time + 1.0 / 6.0 * req_jerk * cycle_time * cycle_time * cycle_time;
            curr_vel = curr_vel + curr_acc * cycle_time + 0.5 * req_jerk * cycle_time * cycle_time;
            curr_acc = curr_acc + req_jerk * cycle_time;
            //next_cycle
            next_progress = progress + curr_vel * cycle_time + 0.5 * curr_acc * cycle_time * cycle_time + 1.0 / 6.0 * req_jerk * cycle_time * cycle_time * cycle_time;
            next_vel = curr_vel + curr_acc * cycle_time + 0.5 * req_jerk * cycle_time * cycle_time;
            next_accel = curr_acc + req_jerk * cycle_time;
            //1.检查加速是否为0
            if (next_accel > 1e-6)
            {
                curr_acc = 0;
                // if (((target - progress) / curr_vel) > 2)
                {
                    accel_state = ACCEL_S3;
                    break;
                }
            }
            //2.限制速度小于0
            if (next_vel < 0)
            {
                curr_vel = tc_param.limit_ve + tc_param.limit_acc * cycle_time;
                curr_acc = 0;
                accel_state = ACCEL_S3;
                break;
            }
            //3.平台插补修正
            if (curr_vel < tc_param.limit_vm)
            {
                if (curr_vel <= turnvel)
                {
                    save_state = accel_state;
                    accel_state = ACCEL_S7;
                    break;
                }
            }
            break;

        case ACCEL_S7:
            progress = progress + curr_vel * cycle_time;
            accel_state = save_state;
            turnvel = 0;
            break;
        default:
            accel_state = ACCEL_S3;
            break;
        }
    }
    if (progress >= target)
    { //添加一个精度误差
        progress = target;
        curr_vel = tc_param.limit_ve;
        curr_acc = 0;
        tc_done = true;
    }
    return;
}

void TrajectoryCalc::tcRunCycleQuintic(TcRunParam &tc_param)
{
    quintic_time += cycle_time;
    if (quintic_time >= 2.0)
    {
        tc_done = true;
        quintic_time = 2.0;
    }
    double posit, vel, acc;
    quinticComputeResult(quintic_factor, quintic_time, posit, vel, acc);
    progress = posit;
    curr_vel = vel;
    curr_acc = acc;
    return;
}

//warning:若末速度不为0,变速后可能调整末速度;
void TrajectoryCalc::tcRunCycleQuinticS(TcRunParam &tc_param)
{

    double dist;
    double k1, k2; //加减速度放大系数
    k1 = 1.3;
    k2 = 1.3; //TODO:加速度放大系数待详细处理
    switch (accel_state)
    {
    case ACCEL_S0:
        quintic_time += cycle_time;
        quinticComputeResult(q_factor, quintic_time, progress, curr_vel, curr_acc);
        if (quintic_time >= toltal_time - 1e-8)
        {
            quinticComputeResult(q_factor, toltal_time, progress, curr_vel, curr_acc);
            accel_state = ACCEL_S3;
            quintic_time = 0;
        }
        break;

    case ACCEL_S3:
        progress = progress + curr_vel * cycle_time;
        //当前到终点路程
        if (tc_param.limit_vm > 1e-3)
        {
            if ((req_acca > (tc_param.limit_vm * acc_vel_rate)))
                k1 = (acc_vel_rate * k1 * tc_param.limit_vm) / req_acca;
            if ((req_accd > (tc_param.limit_vm * dec_vel_rate)))
                k2 = (dec_vel_rate * k2 * tc_param.limit_vm) / req_accd;
            if (k1 < 0.13)
                k1 = 0.13;
            if (k2 < 0.13)
                k2 = 0.13;
        }
        else
        {
            k1 = 1.3;
            k2 = 1.3;
        }
        if (fabs(tc_param.limit_ve - curr_vel) > 1e-5)
        {
            target_vel = tc_param.limit_ve;
            toltal_time = fabs(2 * (target_vel - curr_vel) / (req_accd * k2)); //加速度注意
            toltal_time = ceil(toltal_time / cycle_time);
            if (toltal_time <= 1)
                toltal_time = 11;
            toltal_time = toltal_time * cycle_time;
            dist = toltal_time * (target_vel + curr_vel) / 2;
            if (target - progress <= dist && dist > 0)
            {
                temp_Blend_vel = curr_vel;
                target_vel = tc_param.limit_ve;
                toltal_time = fabs(2 * (target - progress) / (target_vel + curr_vel)); //加速度注意
                toltal_time = ceil(toltal_time / cycle_time);
                if (toltal_time <= 1)
                    toltal_time = 11;
                toltal_time = toltal_time * cycle_time;
                quintic_time = 0;
                q_param.p0 = progress;
                q_param.p1 = target;
                q_param.dp0 = curr_vel;
                q_param.dp1 = target_vel;
                q_param.ddp0 = 0;
                q_param.ddp1 = 0;
                quinticComputeFactor(q_param, q_factor, toltal_time);
                accel_state = ACCEL_S6;
                break;
            }
        }
        //匀速插补(当前速度等于末速度且,目标速度等于末速度)
        if ((fabs(curr_vel - tc_param.limit_ve) < 1e-2) && (fabs(tc_param.limit_vm - tc_param.limit_ve) < 1e-2) &&
            fabs(curr_vel) > 1e-8 && fabs(tc_param.limit_ve > 1e-2))
        {
            target_vel = tc_param.limit_ve;
            dist = target - progress;
            toltal_time = floor(dist / (curr_vel * cycle_time));
            toltal_time = toltal_time * cycle_time;
            quintic_time = 0;
            q_param.p0 = progress;
            q_param.p1 = target;
            q_param.dp0 = curr_vel;
            q_param.dp1 = curr_vel;
            q_param.ddp0 = 0;
            q_param.ddp1 = 0;
            quinticComputeFactor(q_param, q_factor, toltal_time);
            accel_state = ACCEL_S6;
            break;
        }

        //加速,检测最大速度
        if (curr_vel + 1e-5 < tc_param.limit_vm)
        {
            //检查目标速度能否达到
            dist = target - progress;
            target_vel = k1 * k2 * dist * req_acca * req_accd + tc_param.limit_ve * tc_param.limit_ve * k1 * req_acca + curr_vel * curr_vel * k2 * req_accd;
            target_vel = sqrt(target_vel / (fabs(k1 * req_acca) + fabs(k2 * req_accd)));
            if (tc_param.limit_vm <= target_vel)
            {
                target_vel = tc_param.limit_vm;
            }
            if (target_vel >= curr_vel)
            {
                toltal_time = fabs(2 * (target_vel - curr_vel) / (req_acca * k1)); //加速度注意
                toltal_time = ceil(toltal_time / cycle_time);
                if (toltal_time <= 1)
                    toltal_time = 9;
                toltal_time = toltal_time * cycle_time;
                quintic_time = 0;
                q_param.p0 = progress;
                q_param.p1 = progress + toltal_time * (target_vel + curr_vel) / 2;
                q_param.dp0 = curr_vel;
                q_param.dp1 = target_vel;
                q_param.ddp0 = 0;
                q_param.ddp1 = 0;
                quinticComputeFactor(q_param, q_factor, toltal_time);
                accel_state = ACCEL_S0;
                break;
            }
        }
        //减速,调节终点速度
        else if (curr_vel - 1e-8 > tc_param.limit_vm)
        {
            dist = target - progress;
            target_vel = tc_param.limit_vm;
            if (dist < ((curr_vel * curr_vel - target_vel * target_vel) / (k2 * req_accd)))
            {
                temp_Blend_vel = curr_vel;
                req_ve = sqrt(curr_vel * curr_vel - (dist * req_accd * k2));
                target_vel = req_ve;
                toltal_time = fabs(2 * (target - progress) / (target_vel + curr_vel)); //加速度注意
                toltal_time = ceil(toltal_time / cycle_time);
                if (toltal_time <= 1)
                    toltal_time = 9;
                toltal_time = toltal_time * cycle_time;
                quintic_time = 0;
                q_param.p0 = progress;
                q_param.p1 = target;
                q_param.dp0 = curr_vel;
                q_param.dp1 = target_vel;
                q_param.ddp0 = 0;
                q_param.ddp1 = 0;
                quinticComputeFactor(q_param, q_factor, toltal_time);
                accel_state = ACCEL_S6;
                break;
            }
            else
            {
                temp_Blend_vel = curr_vel;
                toltal_time = fabs(2 * (target_vel - curr_vel) / (req_accd * k2)); //加速度注意
                toltal_time = ceil(toltal_time / cycle_time);
                if (toltal_time <= 1)
                    toltal_time = 9;
                toltal_time = toltal_time * cycle_time;
                quintic_time = 0;
                q_param.p0 = progress;
                q_param.p1 = progress + toltal_time * (target_vel + curr_vel) / 2;
                q_param.dp0 = curr_vel;
                q_param.dp1 = target_vel;
                q_param.ddp0 = 0;
                q_param.ddp1 = 0;
                quinticComputeFactor(q_param, q_factor, toltal_time);
                accel_state = ACCEL_S6;
                break;
            }
        }

        break;
    case ACCEL_S6:
        quintic_time += cycle_time;
        quinticComputeResult(q_factor, quintic_time, progress, curr_vel, curr_acc);
        if (quintic_time >= toltal_time - 1e-8)
        {
            quinticComputeResult(q_factor, toltal_time, progress, curr_vel, curr_acc);
            accel_state = ACCEL_S3;
            quintic_time = 0;
        }
        break;
    default:
        accel_state = ACCEL_S3;
        break;
    }
    if ((target - progress < 1e-6) && accel_state == ACCEL_S3)
    {
        if (target - progress < 0)
            progress_error = progress - target;
        tc_done = true;
        progress = target;
    }
    if (((target_vel == tc_param.limit_ve || target_vel == req_ve) && accel_state == ACCEL_S6) || target == progress)
    {
        on_final_dec = true;
    }
    else
    {
        on_final_dec = false;
    }
    return;
}

void TrajectoryCalc::tcRunCycleQuintic_aloha(TcRunParam &tc_param)
{
    double k = 1.0645; //加减速度放大系数
    double t1_min, Vi_max;
    double temp_progress;
    int n_ew = 0; //新的速度
                  //-----参数初始化------------
    if (first == false)
    {
        t1_min = sqrt(2 * target / req_accd);
        Vi_max = t1_min * req_accd;
        target_vel = tc_param.limit_vm;
        if (req_vs < tc_param.limit_vm && tc_param.limit_vm > tc_param.limit_ve)
            n_ew = sqrt(fabs(req_acca * req_accd * target * k + req_accd * req_vs * req_vs + req_acca * tc_param.limit_ve * tc_param.limit_ve) / (req_acca + req_accd));
        else if (req_vs > tc_param.limit_vm && tc_param.limit_vm < tc_param.limit_ve)
            n_ew = sqrt(fabs(req_acca * req_vs * req_vs + req_accd * tc_param.limit_ve * tc_param.limit_ve - req_acca * req_accd * target * k) / (req_accd + req_acca));

        if (req_vs >= Vi_max)
        {
            req_vs = Vi_max;
            S1 = target;
            S2 = 0;
            S3 = 0;
            tc_param.limit_vm = 0;
            tc_param.limit_ve = 0;
            t1 = t1_min;
            t2 = 0;
            t3 = 0;
        }
        else //时间计算
        {
            if (n_ew < 0) //??
            {
                S1 = target;
                if (tc_param.limit_ve > req_vs)
                {
                    tc_param.limit_ve = sqrt((req_acca * target * k) + req_vs * req_vs);
                    t1 = 2 * (tc_param.limit_ve - req_vs) / req_acca / k;
                }
                else
                {
                    tc_param.limit_ve = sqrt(req_vs * req_vs - (req_accd * target * k));
                    t1 = 2 * (tc_param.limit_ve - req_vs) / req_accd / k;
                }
                S2 = 0;
                S3 = 0;
                t2 = 0;
                t3 = 0;
            }
            else
            {
                if (tc_param.limit_vm > req_vs)
                {
                    if (tc_param.limit_vm > n_ew && n_ew > 0)
                    {
                        target_vel = n_ew;
                    }
                }
                else
                {
                    if (tc_param.limit_vm < n_ew && n_ew > 0)
                    {
                        target_vel = tc_param.limit_vm;
                    }
                }
                if (target_vel > req_vs)
                {
                    t1 = 2 * fabs(target_vel - req_vs) / (req_acca * k);
                    S1 = target_vel * t1 + (req_vs - target_vel) * t1 / 2;
                }
                else
                {
                    t1 = 2 * fabs(target_vel - req_vs) / (req_accd * k);
                    S1 = target_vel * t1 + (req_vs - target_vel) * t1 / 2;
                }
                if (target_vel < tc_param.limit_ve)
                {
                    t3 = 2 * fabs(target_vel - tc_param.limit_ve) / (req_acca * k);
                }
                else
                {
                    t3 = 2 * fabs(target_vel - tc_param.limit_ve) / (req_accd * k);
                }
                S3 = target_vel * t3 + (tc_param.limit_ve - target_vel) * t3 / 2;
                S2 = target - S1 - S3;
                if (target_vel != 0)
                {
                    t2 = fabs(S2 / target_vel);
                }
                else
                {
                    if (S2 != 0)
                    {
                        target_vel = tc_param.limit_ve;
                        if (target_vel > req_vs)
                        {
                            t1 = 2 * fabs(target_vel - req_vs) / (req_acca * k);
                            S1 = target_vel * t1 + (req_vs - target_vel) * t1 / 2;
                        }
                        else
                        {
                            t1 = 2 * fabs(target_vel - req_vs) / (req_accd * k);
                            S1 = target_vel * t1 + (req_vs - target_vel) * t1 / 2;
                        }
                        if (target_vel < tc_param.limit_ve)
                        {
                            t3 = 2 * fabs(target_vel - tc_param.limit_ve) / (req_acca * k);
                        }
                        else
                        {
                            t3 = 2 * fabs(target_vel - tc_param.limit_ve) / (req_accd * k);
                        }
                        S3 = target_vel * t3 + (tc_param.limit_ve - target_vel) * t3 / 2;
                        S2 = target - S1 - S3;
                        t2 = fabs(S2 / target_vel);
                    }
                    else
                    {
                        t2 = 0;
                    }
                }
                if (S1 == 0)
                {
                    S1 = S2;
                    t1 = t2;
                    S2 = 0;
                    t2 = 0;
                }
                if (S3 == 0)
                {
                    S3 = S2;
                    t3 = t2;
                    S2 = 0;
                    t2 = 0;
                }
            }
        }
        toltal_time = t1 + t2 + t3;
    }
    first = true;
    //分段控制-------------------------------------------------------
    // 初始化
    if (t == 0)
    {
        temp_progress = 0;
        curr_vel = req_vs;
        curr_acc = 0;
    }
    // 加速段
    if (t > 0 && t <= t1)
    {
        temp_progress = req_vs * t + (pow(t, 6) * (12 * target_vel - 12 * req_vs)) / (12 * pow(t1, 5)) + pow(t, 4) * (20 * target_vel - 20 * req_vs) / (8 * pow(t1, 3)) - (pow(t, 5) * (30 * target_vel - 30 * req_vs)) / (10 * pow(t1, 4));
        curr_vel = req_vs + (pow(t, 5) * (12 * target_vel - 12 * req_vs)) / (2 * pow(t1, 5)) + (pow(t, 3) * (20 * target_vel - 20 * req_vs)) / (2 * pow(t1, 3)) - (pow(t, 4) * (30 * target_vel - 30 * req_vs)) / (2 * pow(t1, 4));
        curr_acc = (5 * pow(t, 4) * (12 * target_vel - 12 * req_vs)) / (2 * pow(t1, 5)) + (3 * t * t * (20 * target_vel - 20 * req_vs)) / (2 * pow(t1, 3)) - (2 * pow(t, 3) * (30 * target_vel - 30 * req_vs)) / (pow(t1, 4));
    }
    // 匀速段
    if (t > t1 && t <= (t1 + t2))
    {
        temp_progress = target_vel * t - (target_vel - req_vs) * t1 / 2;
        curr_vel = target_vel;
        curr_acc = 0;
    }
    //减速段
    if (t > (t1 + t2))
    {
        temp_progress = target_vel * t + (pow((t - t1 - t2), 6) * (12 * req_ve - 12 * target_vel)) / (12 * pow((toltal_time - t1 - t2), 5)) + (pow((t - t1 - t2), 4) * (20 * req_ve - 20 * target_vel)) / (8 * pow((toltal_time - t1 - t2), 3)) - (pow((t - t1 - t2), 5) * (30 * req_ve - 30 * target_vel)) / (10 * pow((toltal_time - t1 - t2), 4)) + (req_vs - target_vel) * t1 / 2;
        curr_vel = target_vel + (pow((t - t1 - t2), 5) * (6 * req_ve - 6 * target_vel)) / (pow((toltal_time - t1 - t2), 5)) + (pow((t - t1 - t2), 3) * (10 * req_ve - 10 * target_vel)) / (pow((toltal_time - t1 - t2), 3)) - (pow((t - t1 - t2), 4) * (15 * req_ve - 15 * target_vel)) / (pow((toltal_time - t1 - t2), 4));
        curr_acc = (5 * pow((t - t1 - t2), 4) * (12 * req_ve - 12 * target_vel)) / (2 * pow((toltal_time - t1 - t2), 5)) + (3 * (t - t1 - t2) * (t - t1 - t2) * (20 * req_ve - 20 * target_vel)) / (2 * pow((toltal_time - t1 - t2), 3)) - (2 * pow((t - t1 - t2), 3) * (30 * req_ve - 30 * target_vel)) / pow((toltal_time - t1 - t2), 4);
    }
    //结束
    progress = Sb + temp_progress;
    // if (t >= toltal_time)
    // {
    //     progress = Sb + target;
    // }
    //变速------------------------------------------------------------------
    if ((t >= t1 && curr_vel != tc_param.limit_vm && t <= (t1 + t2)) || (curr_vel == 0 && curr_vel != tc_param.limit_vm && progress < Sfi))
    {
        target = Sfi - progress;
        t = 0;
        Vb = curr_vel;
        acc1b = curr_acc;
        tb = t;
        Sb = progress;
        target_vel = tc_param.limit_vm;
        t1_min = sqrt(2 * target / req_accd);
        Vi_max = t1_min * req_accd;
        if (curr_vel < tc_param.limit_vm && tc_param.limit_vm > req_ve)
            n_ew = sqrt(fabs(req_acca * req_accd * target * k + req_accd * curr_vel * curr_vel + req_acca * req_ve * req_ve) / (req_acca + req_accd));
        else if (curr_vel > tc_param.limit_vm && tc_param.limit_vm < req_ve)
            n_ew = sqrt(fabs(req_acca * curr_vel * curr_vel + req_accd * req_ve * req_ve - req_acca * req_accd * target * k) / (req_accd + req_acca));
        if (curr_vel >= Vi_max)
        {
            curr_vel = Vi_max;
            S1 = target;
            S2 = 0;
            S3 = 0;
            target_vel = 0;
            req_ve = 0;
            t1 = t1_min;
            t2 = 0;
            t3 = 0;
        }
        else
        {
            if (n_ew < 0)
            {
                S1 = target;
                if (req_ve > curr_vel)
                {
                    req_ve = sqrt((req_acca * target * k) + curr_vel * curr_vel);
                    t1 = 2 * (req_ve - curr_vel) / req_acca / k;
                }
                else
                {
                    req_ve = sqrt(curr_vel * curr_vel - (req_accd * target * k));
                    t1 = 2 * (req_ve - curr_vel) / req_accd / k;
                }
                S2 = 0;
                S3 = 0;
                t2 = 0;
                t3 = 0;
            }
            else
            {
                if (tc_param.limit_vm > curr_vel && tc_param.limit_vm > n_ew && n_ew > 0)
                    target_vel = n_ew;
                else if (tc_param.limit_vm < n_ew && n_ew > 0)
                    target_vel = tc_param.limit_vm;

                if (target_vel > curr_vel)
                {
                    t1 = 2 * fabs(target_vel - curr_vel) / req_acca / k;
                    S1 = target_vel * t1 + (curr_vel - target_vel) * t1 / 2;
                }
                else
                {
                    t1 = 2 * fabs(target_vel - curr_vel) / req_accd / k;
                    S1 = target_vel * t1 + (curr_vel - target_vel) * t1 / 2;
                }
                if (target_vel < req_ve)
                    t3 = 2 * fabs(target_vel - req_ve) / req_acca / k;
                else
                    t3 = 2 * fabs(target_vel - req_ve) / req_accd / k;
                S3 = target_vel * t3 + (req_ve - target_vel) * t3 / 2;
                S2 = target - S1 - S3;
                if (target_vel != 0)
                    t2 = fabs(S2 / target_vel);
                else
                {
                    if (S2 != 0)
                    {
                        target_vel = req_ve;
                        if (target_vel > curr_vel)
                        {
                            t1 = 2 * fabs(target_vel - curr_vel) / req_acca / k;
                            S1 = target_vel * t1 + (curr_vel - target_vel) * t1 / 2;
                        }
                        else
                        {
                            t1 = 2 * fabs(target_vel - curr_vel) / req_accd / k;
                            S1 = target_vel * t1 + (curr_vel - target_vel) * t1 / 2;
                        }
                        if (target_vel < req_ve)
                        {
                            t3 = 2 * fabs(target_vel - req_ve) / req_acca / k;
                        }
                        else
                        {
                            t3 = 2 * fabs(target_vel - req_ve) / req_accd / k;
                        }
                        S3 = target_vel * t3 + (req_ve - target_vel) * t3 / 2;
                        S2 = target - S1 - S3;
                        if (target_vel == 0)
                        {
                            t2 = 0;
                            target = S1;
                        }

                        else
                            t2 = fabs(S2 / target_vel);
                    }
                    else
                        t2 = 0;
                }
                if (S1 == 0)
                {
                    S1 = S2;
                    t1 = t2;
                    S2 = 0;
                    t2 = 0;
                }
                if (S3 == 0)
                {
                    S3 = S2;
                    t3 = t2;
                    S2 = 0;
                    t2 = 0;
                }
            }
            toltal_time = t1 + t2 + t3;
            req_vs = curr_vel;
        }
        // t = t + tb;
        // progress = progress + Sb;
    }
    //判断路程是否结束-------------------------------------------------
    if (progress >= Sfi - 1e-8)
    {
        progress = Sfi;
        tc_done = true;
    }
    t = t + cycle_time;
    if (t > toltal_time)
        t = toltal_time;
    return;
}