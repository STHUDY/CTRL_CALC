#include <windows.h>
#include <stdbool.h>
#include <stdio.h>

#include "ctrl_pid_incremental.h"

ctrl_pid_incremental_t pid;

int main()
{
    ctrl_pid_incremental_init(&pid);
    ctrl_pid_incremental_set_kp(&pid, 1.0f);
    ctrl_pid_incremental_set_ki(&pid, 0.1f);
    ctrl_pid_incremental_set_kd(&pid, 0.01f);

    ctrl_pid_incremental_extra_set_is_sum_out(&pid.extra, ctrl_pid_incremental_true);

    float aimvalue = 0;
    float error = -10;
    float add = -1;

    // PID_Position_CalcResult_NowTureValueIncrementalByNowTureValueAndSpeed(&pidMultipleTest, 0, 0, 0, 0);

    while (true)
    {
        add += rand() / 100000.0f;
        error = -add;

        ctrl_pid_incremental_submit_target(&pid, aimvalue);
        ctrl_pid_incremental_submit_current(&pid, error);

        float result = 0;
        result = ctrl_pid_incremental_update_by_formula_none(&pid, ctrl_pid_incremental_nullptr);
        // float result = PID_Incremental_CalcResult_ByNowTureValue(&pidMultipleTest, error, aimvalue);
        // printf("result:%f\r\n", result);
        printf("error:%f result:%f \r\n", error, result);

        if (add > 30)
        {
            add = -1;
        }

        // if (result > 0) {
        //	result = -result;
        // }

        Sleep(100);
    }
}