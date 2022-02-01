
package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.lang.*;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
import org.firstinspires.ftc.teamcode.Hardware.ChampBot;


@TeleOp(name="TankDriveBernardo", group="ChampBot")
public class TankDriveBernardo extends OpMode {
    ChampBot robot = new ChampBot();
    public ElapsedTime runtime = new ElapsedTime();
    int lowVertPos = 0;
    int midVertPos = 0;
    int highVertPos = 0;
    int leftBound = 0;
    int midBound = 0;
    int rightBound = 0;

    @Override
    public void init() {
        robot.init(hardwareMap);
        final int startingVertPos = robot.ArmMotorVert.getCurrentPosition();
        final int startingHorPos = robot.ArmMotorVert.getCurrentPosition();


        lowVertPos = startingVertPos + 0;//difference in mov
        midVertPos = startingVertPos + 2 * 0;
        highVertPos = startingVertPos + 3 * 0;

        leftBound = startingHorPos - 0;
        midBound = startingHorPos;
        rightBound = startingHorPos + 0;

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
        if (gamepad1.left_trigger > 0 && gamepad1.right_trigger == 0) {
            robot.ArmMotorVert.setPower(-.3);
        }else if (gamepad1.right_trigger > 0 && gamepad1.left_trigger == 0) {
            robot.ArmMotorVert.setPower(.3);
        }else {
            robot.ArmMotorVert.setPower(0);
        }

        if (gamepad1.left_bumper && !gamepad1.right_bumper) {
            robot.ArmMotorHor.setPower(.3);
        }else if (gamepad1.right_bumper && !gamepad1.left_bumper) {
            robot.ArmMotorHor.setPower(-.3);
        }else {
            robot.ArmMotorHor.setPower(0);
        }
        /*
        if (gamepad1.right_trigger > 0 && gamepad1.left_trigger == 0 && robot.currVertPos == 0) {
            robot.ArmMotorVert.setTargetPosition(lowVertPos);
            robot.ArmMotorVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.ArmMotorVert.setPower(0.3);
            while (robot.ArmMotorVert.isBusy()) {
            }
            robot.ArmMotorVert.setPower(0);
            robot.currVertPos = 1;
        } else if (gamepad1.right_trigger > 0 && gamepad1.left_trigger == 0 && robot.currVertPos == 1) {
            robot.ArmMotorVert.setTargetPosition(midVertPos);
            robot.ArmMotorVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.ArmMotorVert.setPower(0.3);
            while (robot.ArmMotorVert.isBusy()) {
            }
            robot.ArmMotorVert.setPower(0);
            robot.currVertPos = 2;
        } else if (gamepad1.right_trigger > 0 && gamepad1.left_trigger == 0 && robot.currVertPos == 2) {
            robot.ArmMotorVert.setTargetPosition(highVertPos);
            robot.ArmMotorVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.ArmMotorVert.setPower(0.3);
            while (robot.ArmMotorVert.isBusy()) {
            }
            robot.ArmMotorVert.setPower(0);
            robot.currVertPos = 3;
        }

        if (gamepad1.left_trigger > 0 && gamepad1.right_trigger == 0 && robot.currVertPos == 3) {
            robot.ArmMotorVert.setTargetPosition(midVertPos);
            robot.ArmMotorVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.ArmMotorVert.setPower(-0.3);
            while (robot.ArmMotorVert.isBusy()) {
            }
            robot.ArmMotorVert.setPower(0);
            robot.currVertPos = 2;
        } else if (gamepad1.left_trigger > 0 && gamepad1.right_trigger == 0 && robot.currVertPos == 2) {
            robot.ArmMotorVert.setTargetPosition(lowVertPos);
            robot.ArmMotorVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.ArmMotorVert.setPower(-0.3);
            while (robot.ArmMotorVert.isBusy()) {
            }
            robot.ArmMotorVert.setPower(0);
            robot.currVertPos = 1;
        } else if (gamepad1.left_trigger > 0 && gamepad1.right_trigger == 0 && robot.currVertPos == 1) {
            robot.ArmMotorVert.setTargetPosition(startingVertPos);
            robot.ArmMotorVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.ArmMotorVert.setPower(-0.3);
            while (robot.ArmMotorVert.isBusy()) {
            }
            robot.ArmMotorVert.setPower(0);
            robot.currVertPos = 0;
        }

        // arm rotate
        if (!gamepad1.left_bumper && gamepad1.right_bumper && robot.currHorPos != 0) {
            robot.ArmMotorHor.setTargetPosition(startingHorPos);
            robot.ArmMotorHor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.ArmMotorHor.setPower(.3);
            while (robot.ArmMotorHor.isBusy()) {
            }
            robot.ArmMotorHor.setPower(0);
            robot.currHorPos = 0;
        } else if (gamepad1.left_bumper && !gamepad1.right_bumper && robot.currHorPos != -1) {
            robot.ArmMotorHor.setTargetPosition(leftBound);
            robot.ArmMotorHor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.ArmMotorHor.setPower(-.3);
            while (robot.ArmMotorHor.isBusy()) {
            }
            robot.ArmMotorHor.setPower(0);
            robot.currHorPos = -1;
        } else if (gamepad1.left_bumper && gamepad1.right_bumper && robot.currHorPos != 1) {
            robot.ArmMotorHor.setTargetPosition(rightBound);
            robot.ArmMotorHor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.ArmMotorHor.setPower(.3);
            while (robot.ArmMotorHor.isBusy()) {
            }
            robot.ArmMotorHor.setPower(0);
            robot.currHorPos = 1;
        }

         */
// intake wheel
        /*
        if (gamepad1.dpad_up && !gamepad1.dpad_down) {
            robot.IntakeMotor.setPower(-0.35); //discharge
        } else if (gamepad1.dpad_down && !gamepad1.dpad_up) {
            robot.IntakeMotor.setPower(0.35); //load
        } else {
            robot.IntakeMotor.setPower(0);
        }

         */

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

                if (gamepad1.dpad_right) {
                robot.setMotorPower(.75, -.75, -.75, .75);
                } else if (gamepad1.dpad_left) {
                robot.setMotorPower(-.75, .75, .75, -.75);
                } else if (gamepad1.dpad_up) {
                robot.setMotorPower(.75, .75, .75, .75);
                } else if (gamepad1.dpad_down) {
                robot.setMotorPower(-.75, -.75, -.75, -.75);
                }
                if (!gamepad1.dpad_up && !gamepad1.dpad_down && !gamepad1.dpad_left && !gamepad1.dpad_right) {
                robot.setMotorPower(-DLY, -DRY, -DLY, -DRY);
                }
                telemetry.addData("ArmHor position : ", robot.ArmMotorHor.getCurrentPosition());
                telemetry.addData("ArmVert Position: ", robot.ArmMotorVert.getCurrentPosition());

                telemetry.addData("FL position : ", robot.DriveFrontLeft.getCurrentPosition());
                telemetry.addData("FR Position: ", robot.DriveFrontRight.getCurrentPosition());
                telemetry.addData("BL position : ", robot.DriveBackLeft.getCurrentPosition());
                telemetry.addData("BR Position: ", robot.DriveBackRight.getCurrentPosition());

                }
                }
