
package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.ChampBot;


@TeleOp(name="TankDriveBernardo", group="ChampBot")
public class TankDriveBernardo extends OpMode {
    ChampBot robot = new ChampBot();
    public ElapsedTime runtime = new ElapsedTime();
    public int startingVertPos = 0;
    public int startingHorPos = 0;

    @Override
    public void init() {
        robot.init(hardwareMap);
        startingVertPos = robot.ArmMotorVert.getCurrentPosition();
        startingHorPos = robot.ArmMotorVert.getCurrentPosition();

        robot.DriveFrontLeft.setPower(0);
        robot.DriveFrontRight.setPower(0);
        robot.DriveBackLeft.setPower(0);
        robot.DriveBackRight.setPower(0);

        telemetry.addData("StartingVert: ", startingVertPos);
        telemetry.addData("StartingHor: ", startingHorPos);

        robot.DriveFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.DriveFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.DriveBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.DriveBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.ArmMotorHor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.ArmMotorVert.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }


    @Override
    public void loop() {
        // tank drive: each stick controls one side of the robot
        // dpad for strafing left/right


        double DLY = gamepad1.left_stick_y * .75;
        double DRY = gamepad1.right_stick_y * .75;
        float DLX = Math.abs(gamepad1.left_stick_x);
        float DRX = Math.abs(gamepad1.right_stick_x);

        telemetry.addData("Right: ", DRY);
        telemetry.addData("Left: ", DLY);
        /*if (leftPower == inf && rightPower == inf) {
            leftPower = 1;
            rightPower = 1;
        } else if (leftPower == -inf && rightPower == -inf) {
            leftPower = -1;
            rightPower = -1;
        } else if (leftPower == inf) {
            leftPower = 1;
        } else if (leftPower == -inf) {
            leftPower = -1;
        } else if (rightPower == inf) {
            rightPower = 1;
        } else if (rightPower == -inf) {
            rightPower = -1;
        } else if (leftPower > 1) {
            leftPower = 1;
        } else if (rightPower > 1) {
            rightPower = 1;
        }
            leftPower = (float) (leftPower * 0.75);
            rightPower = (float) (rightPower  *0.75);
            telemetry.addData("Power R: ", rightPower);
            telemetry.addData("Power L: ", leftPower);

*/

        //arm controls
        /*
        if (gamepad2.a && !gamepad2.y ) {
            robot.moveArmVertDown(startingVertPos);
        }else if (gamepad2.y && !gamepad2.a) {
            robot.moveArmVertUp(startingVertPos + 2318);
        }else {
            robot.ArmMotorVert.setPower(0);
        }

         */

        if (gamepad2.left_trigger > 0 && gamepad2.right_trigger == 0) {
            robot.ArmMotorVert.setPower(1);
        }else if (gamepad2.left_trigger == 0 && gamepad2.right_trigger > 0) {
            robot.ArmMotorVert.setPower(-.8);
        }else {
            robot.ArmMotorVert.setPower(0);
        }

        if (gamepad2.dpad_left && !gamepad2.dpad_right) {
            robot.ArmMotorHor.setPower(.5);
        }else if (gamepad2.dpad_right && !gamepad2.dpad_left) {
            robot.ArmMotorHor.setPower(-.5);
        }else {
            robot.ArmMotorHor.setPower(0);
        }

        if (gamepad2.a && !gamepad2.b) {
            robot.CarouselMotor.setPower(-.5);
        }else if (gamepad2.b && !gamepad2.x) {
            robot.CarouselMotor.setPower(.5);
        }else{
            robot.CarouselMotor.setPower(0);
        }



        if (gamepad1.left_stick_y >= .9) {
            DLY = .75;
        } else if (gamepad1.left_stick_y <= -.9) {
            DLY = -.75;
        }
        if (gamepad1.right_stick_y >= .9) {
            DRY = .75;
        } else if (gamepad1.right_stick_y <= -.9) {
            DRY = -.75;
        }

        if (gamepad1.right_bumper) {
            robot.setMotorPower(.75, -.75, -.75, .75);
        } else if (gamepad1.left_bumper) {
            robot.setMotorPower(-.75, .75, .75, -.75);
        }
        if (gamepad2.right_stick_y >= .9) {
            robot.IntakeMotor.setPower(-.55);
        } else if (gamepad2.right_stick_y <= -.9) {
            robot.IntakeMotor.setPower(.55);
        } else {
            robot.IntakeMotor.setPower(0);
        }

        if (!gamepad1.right_bumper && !gamepad1.left_bumper) {
            robot.setMotorPower(-DLY, -DRY, -DLY, -DRY);
        }
        telemetry.addData("ArmHor position : ", robot.ArmMotorHor.getCurrentPosition());
        telemetry.addData("ArmVert Position: ", robot.ArmMotorVert.getCurrentPosition());
        telemetry.addData("FL position : ", robot.DriveFrontLeft.getCurrentPosition());
        telemetry.addData("FR Position: ", robot.DriveFrontRight.getCurrentPosition());
        telemetry.addData("BL position : ", robot.DriveBackLeft.getCurrentPosition());
        telemetry.addData("BR Position: ", robot.DriveBackRight.getCurrentPosition());
        telemetry.addData("TeouchSensor: ", robot.touchSensor.getValue());
    }

}
