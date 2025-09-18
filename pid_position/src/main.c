#include <windows.h>
#include <stdbool.h>
#include <stdio.h>

#include "ctrl_pid_position.h"

ctrl_pid_position_t pid;

int main()
{
    ctrl_pid_position_init(&pid);
    ctrl_pid_position_set_kp(&pid, 1.0f);
    ctrl_pid_position_set_ki(&pid, 0);
    ctrl_pid_position_set_kd(&pid, 0.01f);

    ctrl_pid_position_extra_set_is_sum_out(&pid.extra, ctrl_pid_position_true);

    float aimvalue = 0;
    float error = -10;
    int add = -1;

    // PID_Position_CalcResult_NowTureValueIncrementalByNowTureValueAndSpeed(&pidMultipleTest, 0, 0, 0, 0);

    while (true)
    {
        add = rand() % 100;

        error += add;

        ctrl_pid_position_submit_target(&pid, aimvalue);
        ctrl_pid_position_submit_current(&pid, error);

        float result = 0;
        result = ctrl_pid_position_update_by_formula_none(&pid, ctrl_pid_position_nullptr);
        // float result = PID_Incremental_CalcResult_ByNowTureValue(&pidMultipleTest, error, aimvalue);
        // printf("result:%f\r\n", result);
        printf("error:%f result:%f \r\n", error, result);

        // if (add > 30)
        // {
        //     add = -1;
        // }

        error += result;

        // if (result > 0) {
        //	result = -result;
        // }

        Sleep(100);
    }
}