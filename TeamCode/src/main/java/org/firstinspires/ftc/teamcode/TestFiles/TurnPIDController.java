package org.firstinspires.ftc.teamcode.TestFiles;

import com.qualcomm.robotcore.util.ElapsedTime;

public class TurnPIDController {
    Gyro variables = new Gyro();

    private double targetAngle;
    private double accumulatedError = 0;
    private ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    private double lastTime = 0;
    public TurnPIDController(double target, double p, double i, double d) {
        targetAngle = target;
        variables.kP = p;
        variables.kI = i;
        variables.kD = d;
    }
    public double update(double currentAngle) {
        // P
        double error = targetAngle - currentAngle;
        error %= 360;
        error += 360;
        error %= 360;
        if (error > 180) {
            error -= 360;
        }

        // I
        accumulatedError += error;
        if (Math.abs(error) < 1) {
            accumulatedError = 0;
        }
        accumulatedError = Math.abs(accumulatedError) * Math.signum(error);

        // D
        double slope = 0;
        if (lastTime > 0) {
            slope = (error - lastError)/ (timer.milliseconds() - lastTime);
        }
        lastTime = timer.milliseconds();
        lastError = error;

        //motor power calculations
        double motorPower = 0.1 * Math.signum(error) + 0.9 * Math.tanh(variables.kP * error + variables.kI * accumulatedError + variables.kD * slope);
        return motorPower;
    }
}
