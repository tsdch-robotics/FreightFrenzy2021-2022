package org.firstinspires.ftc.teamcode.Hardware;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Hardware.ChampBot_v2;

@Autonomous(name = "Gyro", group="ChampBot")
public class Gyro extends LinearOpMode {

    ChampBot_v2 robot = new ChampBot_v2();
    private ElapsedTime runtime = new ElapsedTime();

    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        waitForStart();

        turn(90);
        sleep(3000);
        turnTo(-90);

    }
    //set angle back to 0
    public void resetAngle(){
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currAngle = 0;
    }

    //Get current angle
    public double getAngle() {
        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = orientation.firstAngle - lastAngles.firstAngle;

        if (deltaAngle > 180) {
            deltaAngle -= 360;
        } else if (deltaAngle <= -180) {
            deltaAngle +=360;
        }
        currAngle += deltaAngle;
        lastAngles = orientation;
        telemetry.addData("angle: ", orientation.firstAngle);
        return currAngle;
    }

    public double getAbsoluteAngle() {
        return robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    void turnToPid(double targetAngle) {
        TurnPIDController pid = new TurnPIDController(targetAngle, 0.01,0,0.003);
        while (opModeIsActive() && Math.abs(targetAngle - getAbsoluteAngle()) > 1) {
            double motorPower = pid.update(getAbsoluteAngle());
            robot.setMotorPower(motorPower,motorPower,motorPower,motorPower);
        }
        robot.setAllPower(0);
    }

    void turnPID(double degrees) {
        turnToPid(degrees + getAbsoluteAngle());
    }

    //turn a certain # of degrees
    public void turn(double degrees) {

        resetAngle();

        double error = degrees;

        while (opModeIsActive() && Math.abs(error) > 2) {
            double motorPower = (error < 0 ? -0.3 : 0.3);
            robot.setMotorPower(motorPower, motorPower, motorPower, motorPower);
            error = degrees - getAngle();
            telemetry.addData("error: ", error);

        }

        robot.setAllPower(0);

    }

    //turn to a degree
    public void turnTo(double degrees) {
        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double error = degrees - orientation.firstAngle;

        if (error > 180) {
            error -= 360;
        } else if (error < -180) {
            error += 360;
        }

        turn(error);

    }

}
