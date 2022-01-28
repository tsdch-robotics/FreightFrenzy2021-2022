package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.ChampBot_v2;

import java.lang.*;

@Autonomous(name="MotorRun", group="ChampBot")

public class MotorRun extends LinearOpMode {
    ChampBot_v2 robot = new ChampBot_v2();
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor DriveFrontLeft; //:D
    public DcMotor DriveFrontRight;
    public DcMotor DriveBackLeft;
    public DcMotor DriveBackRight;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();

        robot.DriveFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.DriveFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.DriveBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.DriveBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.DriveBackRight.setPower(0.5);
        robot.DriveBackLeft.setPower(0.5);
        robot.DriveFrontRight.setPower(0.5);
        robot.DriveFrontLeft.setPower(0.5);

    }
}
