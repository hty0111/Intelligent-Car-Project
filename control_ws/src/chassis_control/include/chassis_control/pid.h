/*
 * @Description: 
 * @version: v1.0
 * @Author: HTY
 * @Date: 2022-07-02 21:21:32
 */

#pragma once

namespace chassis_control {

class PID {
public:
    PID();
    PID(double kp, double ki, double kd, double max_out, double max_iout, int mode = 0);
    double compute(double reference, double feedback);
    double get_pout();
    double get_iout();
    double get_dout();


private:
    int mode;   // 0：位置式，1：增量式
    double kp, ki, kd;
    double max_out, max_iout;
    double ref, fdb;
    double p_out, i_out, d_out, out;
    double error[3];     // 0：当前误差，1：上一时刻误差，2：上上时刻误差
    static double inline limit_max(double value, double max_value)
    {
        return (value < -max_value) ? -max_value : (value > max_value) ? max_value : value;
    }
};

}

