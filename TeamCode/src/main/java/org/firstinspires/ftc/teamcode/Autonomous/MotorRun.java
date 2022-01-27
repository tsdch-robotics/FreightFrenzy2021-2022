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

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();

        robot.DriveBackRight.setPower(0.5);
        robot.DriveFrontLeft.setPower(0.5);

    }
}
