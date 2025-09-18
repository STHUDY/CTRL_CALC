#ifndef __CTRL_PID_POSITION_H
#define __CTRL_PID_POSITION_H

#define CTRL_PID_POSITION_VERSION "1.0.0"
#define CTRL_PID_POSITION_USE_EXTRA 1
#define CTRL_PID_POSITION_SAFE 0
#define CTRL_PID_POSITION_OPEN_TEST 0

#define ctrl_pid_position_precision_type float
#define ctrl_pid_position_bool unsigned char
#define ctrl_pid_position_true 1
#define ctrl_pid_position_false 0
#define ctrl_pid_position_nullptr (void *)0

#ifdef __cplusplus
extern "C"
{
#endif

#if CTRL_PID_POSITION_USE_EXTRA == 1
    // extra struct

    typedef struct ctrl_pid_position_extra ctrl_pid_position_extra_t;
    struct ctrl_pid_position_extra
    {
        ctrl_pid_position_bool is_sum_out;
        ctrl_pid_position_precision_type sum_result;

        ctrl_pid_position_bool is_limit_out;
        ctrl_pid_position_precision_type limit_out_up;
        ctrl_pid_position_precision_type limit_out_down;

        ctrl_pid_position_bool is_deadarea_out;
        ctrl_pid_position_precision_type deadarea_out_up;
        ctrl_pid_position_precision_type deadarea_out_down;

        // If this feature is enabled, the function will automatically convert the value of the encoder
        ctrl_pid_position_bool is_unit_out;
        ctrl_pid_position_precision_type unit_size;
        ctrl_pid_position_precision_type unit_precision_offset;
    };

#endif

    // base struct

    typedef struct ctrl_pid_position ctrl_pid_position_t;
    struct ctrl_pid_position
    {
        ctrl_pid_position_precision_type kp;
        ctrl_pid_position_precision_type ki;
        ctrl_pid_position_precision_type kd;

        ctrl_pid_position_precision_type error;
        ctrl_pid_position_precision_type error_prev;
        ctrl_pid_position_precision_type error_sum;

        ctrl_pid_position_precision_type target;
        ctrl_pid_position_precision_type current;
        ctrl_pid_position_precision_type result;

#if CTRL_PID_POSITION_USE_EXTRA == 1
        struct ctrl_pid_position_extra extra;
#endif
    };

    // base function

    ctrl_pid_position_bool ctrl_pid_position_init(ctrl_pid_position_t *pPid);
    ctrl_pid_position_bool ctrl_pid_position_set_kp(ctrl_pid_position_t *pPid, ctrl_pid_position_precision_type kp);
    ctrl_pid_position_bool ctrl_pid_position_set_ki(ctrl_pid_position_t *pPid, ctrl_pid_position_precision_type ki);
    ctrl_pid_position_bool ctrl_pid_position_set_kd(ctrl_pid_position_t *pPid, ctrl_pid_position_precision_type kd);
    ctrl_pid_position_bool ctrl_pid_position_submit_target(ctrl_pid_position_t *pPid, ctrl_pid_position_precision_type target);
    ctrl_pid_position_bool ctrl_pid_position_submit_current(ctrl_pid_position_t *pPid, ctrl_pid_position_precision_type current);
    ctrl_pid_position_precision_type ctrl_pid_position_update_by_formula_none(ctrl_pid_position_t *pPid, ctrl_pid_position_bool *pSuccess);

#if CTRL_PID_POSITION_USE_EXTRA == 1
    // extern function
    ctrl_pid_position_bool ctrl_pid_position_extra_init(ctrl_pid_position_extra_t *pPidExtra);

    ctrl_pid_position_bool ctrl_pid_position_extra_set_is_sum_out(ctrl_pid_position_extra_t *pPidExtra, ctrl_pid_position_bool isSumOut);
    ctrl_pid_position_bool ctrl_pid_position_extra_set_is_limit_out(ctrl_pid_position_extra_t *pPidExtra, ctrl_pid_position_precision_type isLimitOut);
    ctrl_pid_position_bool ctrl_pid_position_extra_set_is_deadarea_out(ctrl_pid_position_extra_t *pPidExtra, ctrl_pid_position_bool isDeadareaOut);
    ctrl_pid_position_bool ctrl_pid_position_extra_set_is_unit_out(ctrl_pid_position_extra_t *pPidExtra, ctrl_pid_position_bool isUnitOut);

    ctrl_pid_position_bool ctrl_pid_position_extra_set_limit_up_out(ctrl_pid_position_extra_t *pPidExtra, ctrl_pid_position_precision_type limitOutUp);
    ctrl_pid_position_bool ctrl_pid_position_extra_set_limit_down_out(ctrl_pid_position_extra_t *pPidExtra, ctrl_pid_position_precision_type limitOutDown);

    ctrl_pid_position_bool ctrl_pid_position_extra_set_deadarea_up_out(ctrl_pid_position_extra_t *pPidExtra, ctrl_pid_position_precision_type deadareaOutUp);
    ctrl_pid_position_bool ctrl_pid_position_extra_set_deadarea_down_out(ctrl_pid_position_extra_t *pPidExtra, ctrl_pid_position_precision_type deadareaOutDown);

    ctrl_pid_position_bool ctrl_pid_position_extra_set_unit_size(ctrl_pid_position_extra_t *pPidExtra, ctrl_pid_position_precision_type unitSize);
    ctrl_pid_position_bool ctrl_pid_position_extra_set_unit_precision_offset(ctrl_pid_position_extra_t *pPidExtra, ctrl_pid_position_precision_type unitPrecisionOffset);

#endif

#if CTRL_PID_POSITION_OPEN_TEST == 1
#include <stdio.h>

    void ctrl_pid_position_test_by_formula_none(void);

#endif

#ifdef __cplusplus
}
#endif

#endif
