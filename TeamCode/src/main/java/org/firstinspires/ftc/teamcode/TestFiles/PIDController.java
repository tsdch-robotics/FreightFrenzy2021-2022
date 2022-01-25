package org.firstinspires.ftc.teamcode.TestFiles;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.SerialNumber;

import org.firstinspires.ftc.teamcode.Hardware.ChampBot_v2;

@Autonomous(name="PID", group="ChampBot")

public abstract class PIDController extends LinearOpMode {

    private BNO055IMU imu; //model number of inertial measurement unit in REV Expansion hub

    double integralSum = 0;
    double Kp = 1;
    double Ki = 0;
    double Kd = 0;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    @Override

    public void runOpMode() throws InterruptedException {
        ChampBot_v2 robot = new ChampBot_v2();
        robot.init(hardwareMap);

        waitForStart();

        robot.DriveFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.DriveFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.DriveBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.DriveBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = hardwareMap.get(BNO055IMU.class, (SerialNumber) imu); // added (SerialNumber) as suggested by debugger
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        waitForStart();

        double referenceAngle = Math.toRadians(90);

        while (opModeIsActive()) {
            double power = PIDControl(100, robot.DriveFrontLeft.getCurrentPosition()); //This references one motor, can use multiple as reference if calculate powers for separate motors
            robot.DriveFrontLeft.setPower(power);
            robot.DriveFrontRight.setPower(power);
            robot.DriveBackLeft.setPower(power);
            robot.DriveBackRight.setPower(power);

            power = PIDControl(referenceAngle, imu.getAngularOrientation().firstAngle); //Angle Control
            robot.DriveFrontLeft.setPower(power);
            robot.DriveFrontRight.setPower(power);
            robot.DriveBackLeft.setPower(power);
            robot.DriveBackRight.setPower(power);
        }
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
