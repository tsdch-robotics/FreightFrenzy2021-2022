package org.firstinspires.ftc.teamcode.TestFiles;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.ElapsedTime;


public class PIDController {

    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard;

    public static double Kp;
    public static double Ki;
    public static double Kd;
    double previousError = 0;

    double integralSum = 0;

    ElapsedTime timer = new ElapsedTime();

    public PIDController (double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    public double update (double target, double state) {
        double error = target - state;
        double dt = timer.seconds();
        integralSum += error * dt;
        double derivative = (error - previousError) / dt;
        double output = Kp * error + Ki * integralSum + Kd * derivative;
        previousError = error;
        timer.reset();
        return output;
    }

}
