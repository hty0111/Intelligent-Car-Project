/*
 * @Description: 
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-07-02 21:21:32
 */


#include "pid.h"

using namespace chassis_control;

PID::PID() = default;

PID::PID(double kp, double ki, double kd, double max_out, double max_iout, int mode)
 : kp(kp), ki(ki), kd(kd), max_out(max_out), max_iout(max_iout), mode(mode),
 ref(0), fdb(0), p_out(0), i_out(0), d_out(0), out(0), error{}
{ }

double PID::compute(double reference, double feedback)
{
    ref = reference;
    fdb = feedback;
    error[2] = error[1];
    error[1] = error[0];
    error[0] = ref - fdb;

    if (mode == 0)
    {
        p_out = kp * error[0];
        i_out += ki * error[0];
        i_out = limit_max(i_out, max_iout);
        d_out = kd * (error[0] - error[1]);
        out = p_out + i_out + d_out;
        out = limit_max(out, max_out);
    }
    else if (mode == 1)
    {
        p_out = kp * (error[0] - error[1]);
        i_out = ki * error[0];
        d_out = kd * (error[0] - 2 * error[1] + error[2]);
        out += p_out + i_out + d_out;
        out = limit_max(out, max_out);
    }

    return out;
}

double PID::get_pout()  { return p_out; }

double PID::get_iout()  { return i_out; }

double PID::get_dout()  { return d_out; }

