package org.firstinspires.ftc.teamcode.TestFiles;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config

public class DrivePIDController {
    FtcDashboard dashboard;
    private double targetEncoder;
    public static double kPd, kId, kDd;
    public double accumulatedError = 0;
    private ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    private double lastTime = 0;
    public double error;
    public double slope = 0;
    public DrivePIDController(double target, double p, double i, double d) {
        targetEncoder = target;
        kPd = p;
        kId = i;
        kDd = d;
    }
    public double update(double currentPosition) {
        // P
        error = targetEncoder - currentPosition;

        // I
        accumulatedError += error;
        if (Math.abs(error) < 1) {
            accumulatedError = 0;
        }
        accumulatedError = Math.abs(accumulatedError) * Math.signum(error);

        // D
        if (lastTime > 0) {
            slope = (error - lastError)/ (timer.milliseconds() - lastTime);
            //slope = (error - lastError)/ 0.2;
        }
        lastTime = timer.milliseconds();
        lastError = error;

        //motor power calculations
        double motorPower = 0.1 * Math.signum(error) + 0.9 * Math.tanh(kPd * error + kId * accumulatedError + kDd * slope);
        return motorPower;

    }

}