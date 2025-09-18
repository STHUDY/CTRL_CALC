#include "ctrl_pid_incremental.h"

#define CTRL_PID_INCREMENTAL_ABS_NUMBER_(number) ((number) < 0 ? -(number) : (number))

#if CTRL_PID_INCREMENTAL_SAFE == 0

#define CTRL_PID_INCREMENTAL_CALC_FORMULA_NONE_(kp, ki, kd, bias, lastOneBias, lastTwoBias) \
    ((kp) * ((bias) - (lastOneBias)) + (ki) * (bias) + (kd) * ((bias) - 2 * (lastOneBias) + (lastTwoBias)))

#else

#define CTRL_PID_INCREMENTAL_CALC_FORMULA_NONE_(kp, ki, kd, bias, lastOneBias, lastTwoBias) \
    (((kp) ? ((kp) * ((bias) - (lastOneBias))) : 0) +                                       \
     ((ki) ? ((ki) * (bias)) : 0) +                                                         \
     ((kd) ? ((kd) * ((bias) - 2 * (lastOneBias) + (lastTwoBias))) : 0))

#endif

ctrl_pid_incremental_bool ctrl_pid_incremental_init(ctrl_pid_incremental_t *pPid)
{

#if CTRL_PID_INCREMENTAL_SAFE == 1
    if (pPid == ctrl_pid_incremental_nullptr)
    {
        return ctrl_pid_incremental_false;
    }
#endif

    pPid->kp = 0;
    pPid->ki = 0;
    pPid->kd = 0;

    pPid->error = (ctrl_pid_incremental_precision_type)0;
    pPid->error_prev = (ctrl_pid_incremental_precision_type)0;
    pPid->error_prev_two = (ctrl_pid_incremental_precision_type)0;

    pPid->result = (ctrl_pid_incremental_precision_type)0;

#if CTRL_PID_INCREMENTAL_USE_EXTRA == 1

    return ctrl_pid_incremental_extra_init(&pPid->extra);

#endif

    return ctrl_pid_incremental_true;
}

ctrl_pid_incremental_bool ctrl_pid_incremental_set_kp(ctrl_pid_incremental_t *pPid, ctrl_pid_incremental_precision_type kp)
{

#if CTRL_PID_INCREMENTAL_SAFE == 1
    if (pPid == ctrl_pid_incremental_nullptr)
    {
        return ctrl_pid_incremental_false;
    }
#endif

    pPid->kp = kp;

    return ctrl_pid_incremental_true;
}

ctrl_pid_incremental_bool ctrl_pid_incremental_set_ki(ctrl_pid_incremental_t *pPid, ctrl_pid_incremental_precision_type ki)
{

#if CTRL_PID_INCREMENTAL_SAFE == 1
    if (pPid == ctrl_pid_incremental_nullptr)
    {
        return ctrl_pid_incremental_false;
    }
#endif

    pPid->ki = ki;

    return ctrl_pid_incremental_true;
}

ctrl_pid_incremental_bool ctrl_pid_incremental_set_kd(ctrl_pid_incremental_t *pPid, ctrl_pid_incremental_precision_type kd)
{

#if CTRL_PID_INCREMENTAL_SAFE == 1
    if (pPid == ctrl_pid_incremental_nullptr)
    {
        return ctrl_pid_incremental_false;
    }
#endif

    pPid->kd = kd;

    return ctrl_pid_incremental_true;
}

ctrl_pid_incremental_bool ctrl_pid_incremental_submit_target(ctrl_pid_incremental_t *pPid, ctrl_pid_incremental_precision_type target)
{
#if CTRL_PID_INCREMENTAL_SAFE == 1
    if (pPid == ctrl_pid_incremental_nullptr)
    {
        return ctrl_pid_incremental_false;
    }
#endif

#if CTRL_PID_INCREMENTAL_USE_EXTRA == 1
    if (pPid->extra.is_unit_out == ctrl_pid_incremental_true)
    {
        target = target * pPid->extra.unit_precision_offset;
    }
#endif

    pPid->target = target;

    return ctrl_pid_incremental_true;
}

ctrl_pid_incremental_bool ctrl_pid_incremental_submit_current(ctrl_pid_incremental_t *pPid, ctrl_pid_incremental_precision_type current)
{

#if CTRL_PID_INCREMENTAL_SAFE == 1
    if (pPid == ctrl_pid_incremental_nullptr)
    {
        return ctrl_pid_incremental_false;
    }
#endif

#if CTRL_PID_INCREMENTAL_USE_EXTRA == 1
    if (pPid->extra.is_unit_out == ctrl_pid_incremental_true)
    {
        current = current * pPid->extra.unit_precision_offset / pPid->extra.unit_size;
    }
#endif

    pPid->current = current;

    return ctrl_pid_incremental_true;
}

ctrl_pid_incremental_precision_type ctrl_pid_incremental_update_by_formula_none(ctrl_pid_incremental_t *pPid, ctrl_pid_incremental_bool *pSuccess)
{
#if CTRL_PID_INCREMENTAL_SAFE == 1
    if (pPid == ctrl_pid_incremental_nullptr)
    {
        if (pSuccess != ctrl_pid_incremental_nullptr)
        {
            *pSuccess = ctrl_pid_incremental_false;
        }
        return 0;
    }
#endif
#if CTRL_PID_INCREMENTAL_USE_EXTRA == 1

    if (pPid->extra.is_deadarea_out == ctrl_pid_incremental_true)
    {
        if (pPid->current < pPid->extra.deadarea_out_up && pPid->current > pPid->extra.deadarea_out_down)
        {
            if (pSuccess != ctrl_pid_incremental_nullptr)
            {
                *pSuccess = ctrl_pid_incremental_true;
            }

            if (pPid->extra.is_sum_out == ctrl_pid_incremental_true)
            {
                return pPid->extra.sum_result;
            }
            else
            {
                return pPid->result;
            }
        }
    }

#endif

    ctrl_pid_incremental_precision_type result = 0;

    pPid->error = pPid->target - pPid->current;
    result = CTRL_PID_INCREMENTAL_CALC_FORMULA_NONE_(pPid->kp, pPid->ki, pPid->kd, pPid->error, pPid->error_prev, pPid->error_prev_two);
    pPid->error_prev_two = pPid->error_prev;
    pPid->error_prev = pPid->error;

    pPid->result = result;

#if CTRL_PID_INCREMENTAL_USE_EXTRA == 1
    if (pPid->extra.is_unit_out == ctrl_pid_incremental_true)
    {
        result = result * pPid->extra.unit_size / pPid->extra.unit_precision_offset;
    }
    if (pPid->extra.is_sum_out == ctrl_pid_incremental_true)
    {
        pPid->extra.sum_result = pPid->extra.sum_result + result;
        result = pPid->extra.sum_result;
    }
    if (pPid->extra.is_limit_out == ctrl_pid_incremental_true)
    {
        ctrl_pid_incremental_precision_type limitOutUpAbs = CTRL_PID_INCREMENTAL_ABS_NUMBER_(pPid->extra.limit_out_up);
        ctrl_pid_incremental_precision_type limitOutDownAbs = CTRL_PID_INCREMENTAL_ABS_NUMBER_(pPid->extra.limit_out_down);
        ctrl_pid_incremental_precision_type resultAbs = CTRL_PID_INCREMENTAL_ABS_NUMBER_(result);

        if (resultAbs > limitOutUpAbs)
        {
            result = result > 0 ? limitOutUpAbs : -limitOutUpAbs;
        }
        else if (resultAbs < limitOutDownAbs)
        {
            result = result > 0 ? limitOutDownAbs : -limitOutDownAbs;
        }
    }
#endif

    if (pSuccess != ctrl_pid_incremental_nullptr)
    {
        *pSuccess = ctrl_pid_incremental_true;
    }

    return result;
}
#if CTRL_PID_INCREMENTAL_USE_EXTRA == 1
ctrl_pid_incremental_bool ctrl_pid_incremental_extra_init(ctrl_pid_incremental_extra_t *pPidExtra)
{
#if CTRL_PID_INCREMENTAL_SAFE == 1
    if (pPidExtra == ctrl_pid_incremental_nullptr)
    {
        return ctrl_pid_incremental_false;
    }
#endif

    pPidExtra->is_sum_out = ctrl_pid_incremental_false;
    pPidExtra->is_limit_out = ctrl_pid_incremental_false;
    pPidExtra->is_deadarea_out = ctrl_pid_incremental_false;
    pPidExtra->is_unit_out = ctrl_pid_incremental_false;

    pPidExtra->sum_result = 0;
    pPidExtra->limit_out_up = 0;
    pPidExtra->limit_out_down = 0;
    pPidExtra->deadarea_out_up = 0;
    pPidExtra->deadarea_out_down = 0;
    pPidExtra->unit_size = 1;
    pPidExtra->unit_precision_offset = 1;

    return ctrl_pid_incremental_true;
}

ctrl_pid_incremental_bool ctrl_pid_incremental_extra_set_is_sum_out(ctrl_pid_incremental_extra_t *pPidExtra, ctrl_pid_incremental_bool isSumOut)
{

#if CTRL_PID_INCREMENTAL_SAFE == 1
    if (pPidExtra == ctrl_pid_incremental_nullptr)
    {
        return ctrl_pid_incremental_false;
    }
#endif

    pPidExtra->is_sum_out = isSumOut;

    return ctrl_pid_incremental_true;
}

ctrl_pid_incremental_bool ctrl_pid_incremental_extra_set_is_limit_out(ctrl_pid_incremental_extra_t *pPidExtra, ctrl_pid_incremental_precision_type isLimitOut)
{
#if CTRL_PID_INCREMENTAL_SAFE == 1
    if (pPidExtra == ctrl_pid_incremental_nullptr)
    {
        return ctrl_pid_incremental_false;
    }
#endif

    pPidExtra->is_limit_out = isLimitOut;

    return ctrl_pid_incremental_true;
}

ctrl_pid_incremental_bool ctrl_pid_incremental_extra_set_is_deadarea_out(ctrl_pid_incremental_extra_t *pPidExtra, ctrl_pid_incremental_bool isDeadareaOut)
{
#if CTRL_PID_INCREMENTAL_SAFE == 1
    if (pPidExtra == ctrl_pid_incremental_nullptr)
    {
        return ctrl_pid_incremental_false;
    }
#endif

    pPidExtra->is_deadarea_out = isDeadareaOut;

    return ctrl_pid_incremental_true;
}

ctrl_pid_incremental_bool ctrl_pid_incremental_extra_set_is_unit_out(ctrl_pid_incremental_extra_t *pPidExtra, ctrl_pid_incremental_bool isUnitOut)
{
#if CTRL_PID_INCREMENTAL_SAFE == 1
    if (pPidExtra == ctrl_pid_incremental_nullptr)
    {
        return ctrl_pid_incremental_false;
    }
#endif

    pPidExtra->is_unit_out = isUnitOut;

    return ctrl_pid_incremental_true;
}

ctrl_pid_incremental_bool ctrl_pid_incremental_extra_set_limit_up_out(ctrl_pid_incremental_extra_t *pPidExtra, ctrl_pid_incremental_precision_type limitOutUp)
{
#if CTRL_PID_INCREMENTAL_SAFE == 1
    if (pPidExtra == ctrl_pid_incremental_nullptr)
    {
        return ctrl_pid_incremental_false;
    }
#endif

    pPidExtra->limit_out_up = limitOutUp;

    return ctrl_pid_incremental_true;
}

ctrl_pid_incremental_bool ctrl_pid_incremental_extra_set_limit_down_out(ctrl_pid_incremental_extra_t *pPidExtra, ctrl_pid_incremental_precision_type limitOutDown)
{
#if CTRL_PID_INCREMENTAL_SAFE == 1
    if (pPidExtra == ctrl_pid_incremental_nullptr)
    {
        return ctrl_pid_incremental_false;
    }
#endif

    pPidExtra->limit_out_down = limitOutDown;

    return ctrl_pid_incremental_true;
}

ctrl_pid_incremental_bool ctrl_pid_incremental_extra_set_deadarea_up_out(ctrl_pid_incremental_extra_t *pPidExtra, ctrl_pid_incremental_precision_type deadareaOutUp)
{
#if CTRL_PID_INCREMENTAL_SAFE == 1
    if (pPidExtra == ctrl_pid_incremental_nullptr)
    {
        return ctrl_pid_incremental_false;
    }
#endif

    pPidExtra->deadarea_out_up = deadareaOutUp;

    return ctrl_pid_incremental_true;
}

ctrl_pid_incremental_bool ctrl_pid_incremental_extra_set_deadarea_down_out(ctrl_pid_incremental_extra_t *pPidExtra, ctrl_pid_incremental_precision_type deadareaOutDown)
{
#if CTRL_PID_INCREMENTAL_SAFE == 1
    if (pPidExtra == ctrl_pid_incremental_nullptr)
    {
        return ctrl_pid_incremental_false;
    }
#endif

    pPidExtra->deadarea_out_down = deadareaOutDown;

    return ctrl_pid_incremental_true;
}

ctrl_pid_incremental_bool ctrl_pid_incremental_extra_set_unit_size(ctrl_pid_incremental_extra_t *pPidExtra, ctrl_pid_incremental_precision_type unitSize)
{
#if CTRL_PID_INCREMENTAL_SAFE == 1
    if (pPidExtra == ctrl_pid_incremental_nullptr)
    {
        return ctrl_pid_incremental_false;
    }
#endif

    pPidExtra->unit_size = unitSize;

    return ctrl_pid_incremental_true;
}

ctrl_pid_incremental_bool ctrl_pid_incremental_extra_set_unit_precision_offset(ctrl_pid_incremental_extra_t *pPidExtra, ctrl_pid_incremental_precision_type unitPrecisionOffset)
{
#if CTRL_PID_INCREMENTAL_SAFE == 1
    if (pPidExtra == ctrl_pid_incremental_nullptr)
    {
        return ctrl_pid_incremental_false;
    }
#endif

    pPidExtra->unit_precision_offset = unitPrecisionOffset;

    return ctrl_pid_incremental_true;
}

#endif

#if CTRL_PID_INCREMENTAL_OPEN_TEST == 1
void ctrl_pid_incremental_test_by_formula_none(void)
{
    ctrl_pid_incremental_t pid;

    // 假设一组测试输入误差 (模拟系统偏差序列)
    float error_seq[] = {10, 7, 5, 2, 0, -1, -2};
    int len = sizeof(error_seq) / sizeof(error_seq[0]);

    // 初始化 PID
    ctrl_pid_incremental_init(&pid);
    ctrl_pid_incremental_set_kp(&pid, 0.5f);
    ctrl_pid_incremental_set_ki(&pid, 0.1f);
    ctrl_pid_incremental_set_kd(&pid, 0.05f);

    printf("PID test by formula none\n");
    printf("PID test by formula none, len=%d\n", len);
    printf("PID test by formula none, kp=%.2f, ki=%.2f, kd=%.2f\n", pid.kp, pid.ki, pid.kd);

    float sum = 0;

    // 遍历测试
    for (int i = 0; i < len; i++)
    {
        // 提交目标值与当前值（这里假设目标是 0）
        ctrl_pid_incremental_submit_target(&pid, 0.0f);
        ctrl_pid_incremental_submit_current(&pid, error_seq[i]);

        // 更新并计算 PID 增量
        float delta_u = ctrl_pid_incremental_update_by_formula_none(&pid, ctrl_pid_incremental_nullptr);
        sum += delta_u;

        printf("step=%d, error=%.2f, delta_u=%.3f, result=%.3f, sum=%.3f\n",
               i + 1, pid.error, delta_u, pid.result, sum);
    }
}

#endif
