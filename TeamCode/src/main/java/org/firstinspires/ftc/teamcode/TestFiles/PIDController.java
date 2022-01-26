package org.firstinspires.ftc.teamcode.TestFiles;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.SerialNumber;

import org.firstinspires.ftc.teamcode.Hardware.ChampBot_v2;
@Config
@Autonomous(name="PID", group="ChampBot")

public class PIDController extends LinearOpMode {


    //private BNO055IMU imu; //model number of inertial measurement unit in REV Expansion hub
    ChampBot_v2 robot = new ChampBot_v2();
    public static double integralSum = 0;
    public static double Kp = 1;
    public static double Ki = 0;
    public static double Kd = 0;

    FtcDashboard dashboard;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);


        robot.DriveFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.DriveFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.DriveBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.DriveBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //imu = hardwareMap.get(BNO055IMU.class, (SerialNumber) imu); // added (SerialNumber) as suggested by debugger
        //BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        //parameters.mode = BNO055IMU.SensorMode.IMU;
        //parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        //imu.initialize(parameters);

        dashboard = FtcDashboard.getInstance();

        waitForStart();

        double referenceAngle = Math.toRadians(90);

        while (opModeIsActive()) {

            telemetry.addData("MotorCount: ", robot.DriveFrontLeft.getCurrentPosition());
            sleep(2000);

            double power = PIDControl(1000, robot.DriveFrontLeft.getCurrentPosition()); //This references one motor, can use multiple as reference if calculate powers for separate motors
            robot.DriveFrontLeft.setPower(power);
            robot.DriveFrontRight.setPower(power);
            robot.DriveBackLeft.setPower(power);
            robot.DriveBackRight.setPower(power);

            //power = PIDControl(referenceAngle, imu.getAngularOrientation().firstAngle); //Angle Control
            //robot.DriveFrontLeft.setPower(power);
            //robot.DriveFrontRight.setPower(power);
            //robot.DriveBackLeft.setPower(power);
            //robot.DriveBackRight.setPower(power);
        }
        sleep(2000);
    }

    public double PIDControl(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }
/*
    public double AngleControl(double reference, double imuangle) {
        double error = reference - imuangle;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }

    public double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;
    }
}
*/
}
