package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.TestFiles.PIDController;
import org.firstinspires.ftc.teamcode.TestFiles.PIDControllerHor;

// Vision imports


/*
 * This is NOT an opmode. This file defines all the hardware on the robot
 * and some common helper functions (stop motors, reset encoders, etc.)
 */
public class ChampBot<Directionvector> {
    public static double kvP = 0.3;
    public static double kvI = 0.001;
    public static double kvD = 0.01;
    public static double khP = 0.001;
    public static double khI = 0;
    public static double khD = 0.00001;
    //imu
    public BNO055IMU imu;
    //Drive Motors
    public DcMotor DriveFrontLeft; //:D
    public DcMotor DriveFrontRight;
    public DcMotor DriveBackLeft;
    public DcMotor DriveBackRight;
    public DcMotor IntakeMotor;
    public DcMotorEx ArmMotorVert;
    public DcMotor ArmMotorHor;
    public DcMotor CarouselMotor;
    //File Imports
    PIDController control = new PIDController(kvP, kvI, kvD);
    PIDControllerHor controlHor = new PIDControllerHor(khP, khI, khD);
    //Odometry Encoders
    //public Servo Carousel;
    //Sensors
    public DigitalChannel touchSensor;
    //public ColorSensor color_sensor;
    //code time! :)
    private HardwareMap hardwareMap;
    //Vuforia Key
    public static final String VUFORIA_KEY = "AT3AS3b/////AAABmeb4lXVRG0y7kvB4fG/FtbKNEuekc1o6rkU8fj5JsakaaPPNIV+ZalkosfH2Zl513dKhMTyUF6YtHzQUr07sQDtKs2GbaulwZFC5yov+tEsgEHA+OWVni0f18T87qrboMBnj61MUqZP4BvjPib2R1rxEr3Cj4YS9c9RnkUESyTBI6B/gCtBFLNKh7cX9PKJnKCgkVF9qexL2eu2lRS1iNAl9VsjBel6RSckVjGEo90+DSyVMgUtew2GNk8IokXYlBHlK7MY6ju3Kb1dFyLLm0RPcXxqvA3pIPNbk8Rsuv75xhBGBRMVpgteBaaWZz6YUowvEv5YfGdTjy1kmQyzufcwqvMY038k0BzPCUGXE77Md";

    public void init(HardwareMap ahwMap) {
        hardwareMap = ahwMap;

        touchSensor = hardwareMap.get(DigitalChannel.class,"touchSensor");

        ArmMotorVert = hardwareMap.get(DcMotorEx.class, "ArmMotorVert");
        ArmMotorVert.setDirection(DcMotor.Direction.REVERSE);
        ArmMotorVert.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmMotorVert.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        CarouselMotor = hardwareMap.dcMotor.get("CarouselMotor");
        CarouselMotor.setDirection(DcMotor.Direction.FORWARD);
        CarouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        CarouselMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = (BNO055IMU) hardwareMap.get("imu");
        imu.initialize(parameters);

        DriveFrontLeft = hardwareMap.dcMotor.get("DriveFrontLeft");
        DriveFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        DriveFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DriveFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DriveFrontRight = hardwareMap.dcMotor.get("DriveFrontRight");
        DriveFrontRight.setDirection(DcMotor.Direction.REVERSE);
        DriveFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DriveFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DriveBackLeft = hardwareMap.dcMotor.get("DriveBackLeft");
        DriveBackLeft.setDirection(DcMotor.Direction.FORWARD);
        DriveBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DriveBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DriveBackRight = hardwareMap.dcMotor.get("DriveBackRight");
        DriveBackRight.setDirection(DcMotor.Direction.REVERSE);
        DriveBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DriveBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        IntakeMotor = hardwareMap.dcMotor.get("IntakeMotor");
        IntakeMotor.setDirection(DcMotor.Direction.REVERSE);
        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ArmMotorHor = hardwareMap.dcMotor.get("ArmMotorHor");
        ArmMotorHor.setDirection(DcMotor.Direction.REVERSE);
        ArmMotorHor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmMotorHor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Carousel = hardwareMap.servo.get("Claw");

        //color_sensor = hardwareMap.get(ColorSensor.class, "color_sensor");

        stop();
        resetDriveEncoders();
    }

    //a function to reset encoder
    public void resetDriveEncoders() {
        DriveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void enableEncoders() {
        DriveFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DriveBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    //the stop function

    public void stop() {
        DriveFrontLeft.setPower(0);
        DriveFrontRight.setPower(0);
        DriveBackRight.setPower(0);
        DriveBackLeft.setPower(0);
    }

    public void setMotorPower(double FrontL, double FrontR, double BackL, double BackR) {
        DriveFrontLeft.setPower(FrontL);
        DriveFrontRight.setPower(FrontR);
        DriveBackLeft.setPower(BackL);
        DriveBackRight.setPower(BackR);
    }

    public void setAllPower(double p) {
        setMotorPower(p, p, p, p);
    }


    public void moveArmVertUp(double Target) {
        double target = Target;
        double error = target - ArmMotorVert.getCurrentPosition();
        while (Math.abs(error) > 20) {
            double command = control.update(target, ArmMotorVert.getCurrentPosition());
            ArmMotorVert.setPower(command);
            error = target - ArmMotorVert.getCurrentPosition();
        }
        ArmMotorVert.setPower(0);
    }

    public void moveArmVertDown(double Target) {
        double target = Target;
        double error = target - ArmMotorVert.getCurrentPosition();
        while (touchSensor.getState() == true) {
            double command = control.update(target, ArmMotorVert.getCurrentPosition());
            ArmMotorVert.setPower(command);
            error = target - ArmMotorVert.getCurrentPosition();
        }
        ArmMotorVert.setPower(0);
    }

    public void moveArmHor(double Target) {
        double target = Target;
        double error = target - ArmMotorHor.getCurrentPosition();
        while (Math.abs(error) > 20) {
            double command = controlHor.update(target, ArmMotorHor.getCurrentPosition());
            ArmMotorHor.setPower(command);
            error = target - ArmMotorHor.getCurrentPosition();
        }
        ArmMotorHor.setPower(0);
    }
}