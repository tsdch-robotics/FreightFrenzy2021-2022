package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

// Vision imports


/*
 * This is NOT an opmode. This file defines all the hardware on the robot
 * and some common helper functions (stop motors, reset encoders, etc.)
 */
public class ChampBot_v3 {
    public static final Double TRIGGER_THRESHOLD = 0.5;//gamepad trigger
    //1080 = 270 degrees
    //robot variables
    public double startingEncoderCount = 0.5;
    //Drive Motors
    public DcMotorEx DriveFrontLeft; //:D
    public DcMotorEx DriveFrontRight;
    public DcMotorEx DriveBackLeft;
    public DcMotorEx DriveBackRight;

    public DcMotorEx WheelMotor;
    public DcMotorEx IntakeMotor;
    public DcMotorEx LaunchMotor;

    //Gyro

    //Vuforia Key
    public static final String VUFORIA_KEY = "AT3AS3b/////AAABmeb4lXVRG0y7kvB4fG/FtbKNEuekc1o6rkU8fj5JsakaaPPNIV+ZalkosfH2Zl513dKhMTyUF6YtHzQUr07sQDtKs2GbaulwZFC5yov+tEsgEHA+OWVni0f18T87qrboMBnj61MUqZP4BvjPib2R1rxEr3Cj4YS9c9RnkUESyTBI6B/gCtBFLNKh7cX9PKJnKCgkVF9qexL2eu2lRS1iNAl9VsjBel6RSckVjGEo90+DSyVMgUtew2GNk8IokXYlBHlK7MY6ju3Kb1dFyLLm0RPcXxqvA3pIPNbk8Rsuv75xhBGBRMVpgteBaaWZz6YUowvEv5YfGdTjy1kmQyzufcwqvMY038k0BzPCUGXE77Md";
    //code time! :)
    private HardwareMap hardwareMap;

    public void init(HardwareMap ahwMap) {
        hardwareMap = ahwMap;
        //configure the drive motors

        DriveFrontLeft = hardwareMap.get(DcMotorEx.class, "DriveFrontLeft");
        DriveFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        DriveFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DriveFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DriveFrontRight = hardwareMap.get(DcMotorEx.class, "DriveFrontRight");
        DriveFrontRight.setDirection(DcMotor.Direction.REVERSE);
        DriveFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DriveFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DriveBackLeft = hardwareMap.get(DcMotorEx.class, "DriveBackLeft");
        DriveBackLeft.setDirection(DcMotor.Direction.FORWARD);
        DriveBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DriveBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DriveBackRight = hardwareMap.get(DcMotorEx.class, "DriveBackRight");
        DriveBackRight.setDirection(DcMotor.Direction.REVERSE);
        DriveBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DriveBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        WheelMotor = hardwareMap.get(DcMotorEx.class, "WheelMotor");
        WheelMotor.setDirection(DcMotor.Direction.FORWARD);
        WheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        WheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        IntakeMotor = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
        IntakeMotor.setDirection(DcMotor.Direction.REVERSE);
        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LaunchMotor = hardwareMap.get(DcMotorEx.class, "LaunchMotor");
        LaunchMotor.setDirection(DcMotor.Direction.REVERSE);
        LaunchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LaunchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
/*
        stop();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        */


    }

    //a function to reset encoder
    public void resetDriveEncoders() {
        DriveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DriveBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DriveBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        //WheelMotor.setPower(0);
    }
    public void setAllPower(double p) {
        setMotorPower(p,p,p,p);
    }

    public void setMotorPower(double FrontL, double FrontR, double BackL, double BackR) {
        DriveFrontLeft.setPower(FrontL);
        DriveFrontRight.setPower(FrontR);
        DriveBackLeft.setPower(BackL);
        DriveBackRight.setPower(BackR);
    }
/*
    public void setIntakePower (double pw) {
        IntakeMotor.setPower(pw);
    }

    public void setLaunchPower (double pwr) {
        LaunchMotor.setPower(pwr);
    }
/*
    public void setWheelPower(double WheelP) {
        WheelMotor.setPower(WheelP);
    }
*/
}