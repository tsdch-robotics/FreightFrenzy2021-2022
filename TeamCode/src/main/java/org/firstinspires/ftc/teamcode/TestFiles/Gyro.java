package org.firstinspires.ftc.teamcode.TestFiles;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Hardware.ChampBot_v2;
import java.lang.*;

@Config
@Autonomous(name = "Gyro", group="ChampBot")
public class Gyro extends LinearOpMode {

    ChampBot_v2 robot = new ChampBot_v2();

    FtcDashboard dashboard;

    private ElapsedTime runtime = new ElapsedTime();

    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;

    public static double kP = 5;
    public static double kI = 0;
    public static double kD = 1;
    public DcMotor DriveFrontLeft; //:D
    public DcMotor DriveFrontRight;
    public DcMotor DriveBackLeft;
    public DcMotor DriveBackRight;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        dashboard = FtcDashboard.getInstance();

        waitForStart();

        robot.DriveFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.DriveFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.DriveBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.DriveBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("It started!!!", null);
        telemetry.update();

        turn(90);
        telemetry.addData("Turning right...",null);
        telemetry.update();
        sleep(3000);
        turnTo(-90);
        telemetry.addData("Turning left...",null);
        telemetry.update();
        sleep(3000);
        moveToPid(1000);
        telemetry.addData("Moving...",null);
        telemetry.update();
        sleep(3000);

    }
    //set angle back to 0
    public void resetAngle(){
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
        currAngle = 0;
    }

    //Get current angle
    public double getAngle() {
        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

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
        return robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES).firstAngle;
    }

    void turnToPid(double targetAngle) {
        TurnPIDController pidTurn = new TurnPIDController(targetAngle, 1,0,.5);
        while (opModeIsActive() && Math.abs(targetAngle - getAbsoluteAngle()) > 1) {
            double motorPower = pidTurn.update(getAbsoluteAngle());
            robot.setMotorPower(motorPower,motorPower,motorPower,motorPower);
        }
        robot.setAllPower(0);
    }

    void moveToPid ( double targetEncoder) {
        DrivePIDController pidDrive = new DrivePIDController(targetEncoder, 1, 0 , .5);
        while (opModeIsActive() && Math.abs(targetEncoder - getCurrentPosition()) > 1) {
            double motorPower = pidDrive.update(getCurrentPosition());
            telemetry.addData("motorPower: ", motorPower);
            telemetry.update();
            robot.setMotorPower(motorPower,motorPower,motorPower,motorPower);
        }
        robot.setAllPower(0);
    }

    public double getCurrentPosition() {
        return robot.DriveFrontLeft.getCurrentPosition();
    }

    void turnPID(double degrees) {
        turnToPid(degrees + getAbsoluteAngle());
    }


    public void turnTo(double degrees) {
        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

        double error = degrees - orientation.firstAngle;

        if (error > 180) {
            error -= 360;
        } else if (error < -180) {
            error += 360;
        }

        turn(error);

    }

    //turn a certain # of degrees
    public void turn(double degrees) {

        resetAngle();

        double error = degrees;

        while (opModeIsActive() && Math.abs(error) > 2) {
            double motorPower = (error < 0 ? -0.3 : 0.3);
            //robot.setMotorPower(motorPower, -motorPower, motorPower, -motorPower);
            robot.DriveFrontLeft.setPower(motorPower);
            robot.DriveFrontRight.setPower(-motorPower);
            robot.DriveBackLeft.setPower(motorPower);
            robot.DriveBackRight.setPower(-motorPower);
            error = degrees - getAngle();
            telemetry.addData("error: ", error);
            telemetry.update();

        }

        robot.setAllPower(0);

    }


}
