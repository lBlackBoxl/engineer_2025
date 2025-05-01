#include "PID.h"
#include "main.h"
#include "MATH_LIB.h"
#include "gimbal_task.h"
 #define limit_max(input, max)  \
    {                           \
        if (input > max)        \
        {                       \
            input = max;        \
        }                       \
        else if (input < -max)  \
        {                       \
            input = -max;       \
        }                       \
    }

void PID_init(pid_type_def *pid, uint8_t mode, float32_t kp, float32_t ki, float32_t kd, float32_t max_out, float32_t max_iout)
{
    if(pid == NULL) 
        return;
    pid->mode = mode;
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

float32_t PID_calc_angle(pid_type_def *pid, float32_t ref, float32_t set)
{
    if (pid == NULL) 
    {
        return 0.0f;
    } 
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    pid->error[0] =	rad_format(pid->error[0]);
    if(pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        limit_max(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        limit_max(pid->out, pid->max_out);
    }
    else if(pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        limit_max(pid->out, pid->max_out);
    }
    return pid->out;
}

float32_t PID_calc(pid_type_def *pid, float32_t ref, float32_t set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        limit_max(pid->Iout, pid->max_iout);
        pid->out = (pid->Pout + pid->Iout + pid->Dout);
        limit_max(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += (pid->Pout + pid->Iout + pid->Dout);
        limit_max(pid->out, pid->max_out);
    }
    return pid->out;
}
//角度环PID初始化
void gimbal_PID_init(pid_type_def *pid, float32_t maxout, float32_t max_iout, float32_t kp, float32_t ki, float32_t kd)
{
    if (pid == NULL)
    {
        return;
    }
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->err = 0.0f;
    pid->get = 0.0f;
    pid->max_iout = max_iout;
    pid->max_out = maxout;
}
//角度环PID计算
float32_t gimbal_PID_calc(pid_type_def *pid, float32_t get, float32_t set, float32_t error_delta)
{
    float32_t err;
    if (pid == NULL)
    {
        return 0.0f;
    }
	pid->get = get;
    pid->set = set;
    err = set - get;
    pid->err = rad_format(err);
    pid->Pout = pid->Kp * pid->err;
    pid->Iout += pid->Ki * pid->err;
	if(fabs(pid->err) > 0.1f)
		pid->Iout = 0;
    pid->Dout = pid->Kd * error_delta;
    pid->Iout = float32_limit(pid->Iout, -pid->max_iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    pid->out = float32_limit(pid->out, -pid->max_out, pid->max_out);
    return pid->out;
}

//角度环PID清除参数
void gimbal_PID_clear(pid_type_def *gimbal_pid_clear)
{
    if (gimbal_pid_clear == NULL)
    {
        return;
    }
    gimbal_pid_clear->err = gimbal_pid_clear->set = gimbal_pid_clear->get = 0.0f;
    gimbal_pid_clear->out = gimbal_pid_clear->Pout = gimbal_pid_clear->Iout = gimbal_pid_clear->Dout = 0.0f;
}
