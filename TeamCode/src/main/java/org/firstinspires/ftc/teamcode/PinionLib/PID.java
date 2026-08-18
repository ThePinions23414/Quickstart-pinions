package org.firstinspires.ftc.teamcode.PinionLib;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
    public double P;
    public double I;
    public double D;

    public double lastError;
    public double integralSum;

    public double max_i = 0.2, min_i = -0.2;

    public PID(double p, double i, double d){
        P = p;
        I = i;
        D = d;
    }

    public double calculate(double current, double target){
        ElapsedTime time = new ElapsedTime();

        double error = target - current;

        double derivative = (error - lastError) / time.seconds();

        integralSum = integralSum + (error * time.seconds());

        double out = (P * error) + (I * integralSum) + (D * derivative);

        lastError = error;

        return (out);
    };
}
