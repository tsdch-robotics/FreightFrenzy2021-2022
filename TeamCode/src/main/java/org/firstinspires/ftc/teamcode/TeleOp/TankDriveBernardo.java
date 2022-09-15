
package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.ChampBot_v3;


@TeleOp(name="TankDriveBernardo", group="ChampBot")
public class TankDriveBernardo extends OpMode {
    ChampBot_v3 robot = new ChampBot_v3();
    public ElapsedTime runtime = new ElapsedTime();
    public int startingVertPos = 0;
    public int startingHorPos = 0;
    public int CurrPos = 0;

    @Override
    public void init() {
        robot.init(hardwareMap);

        robot.DriveFrontLeft.setPower(0);
        robot.DriveFrontRight.setPower(0);
        robot.DriveBackLeft.setPower(0);
        robot.DriveBackRight.setPower(0);

        // robot.WheelMotor.setPower(0);
        //robot.IntakeMotor.setPower(0);
        //robot.LaunchMotor.setPower(0);

        telemetry.addData("StartingVert: ", startingVertPos);
        telemetry.addData("StartingHor: ", startingHorPos);

        robot.DriveFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.DriveFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.DriveBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.DriveBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }


    @Override
    public void loop() {
        // tank drive: each stick controls one side of the robot
        // dpad for strafing left/right


        double DLY = gamepad1.left_stick_y;
        double DRY = gamepad1.right_stick_y;
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

        if (gamepad1.dpad_right && !gamepad1.dpad_up && !gamepad1.dpad_down && !gamepad1.dpad_left) {
            robot.setMotorPower(.75, -.75, -.75, .75);
        }
        if (gamepad1.dpad_left && !gamepad1.dpad_right && !gamepad1.dpad_up && !gamepad1.dpad_down) {
            robot.setMotorPower(-.75, .75, .75, -.75);
        }
        if (gamepad1.dpad_up && !gamepad1.dpad_down && !gamepad1.dpad_left && !gamepad1.dpad_right) {
            robot.setMotorPower(.75, .75, .75, .75);
        }
        if (gamepad1.dpad_down&& !gamepad1.dpad_up && !gamepad1.dpad_left && !gamepad1.dpad_right) {
            robot.setMotorPower(-.75, -.75, -.75, -.75);
        }

        if (!gamepad1.dpad_left && !gamepad1.dpad_right && !gamepad1.dpad_up && !gamepad1.dpad_down) {
            robot.setMotorPower(-DLY, -DRY, -DLY, -DRY);
        }

        if (gamepad1.x) {
            robot.WheelMotor.setPower(1);
        }
        else if (gamepad1.y) {
            robot.WheelMotor.setPower(-1);
        }
        else {
            robot.WheelMotor.setPower(0);
        }


        if (gamepad1.b) {
            robot.IntakeMotor.setPower(-0.7);
        }
        else {
            robot.IntakeMotor.setPower(0);
        }

        if (gamepad1.right_trigger > 0) {
            robot.LaunchMotor.setPower(-1);
            robot.IntakeMotor.setPower(-1);
        }
        else {
            robot.LaunchMotor.setPower(0);
        }
      if (gamepad1.left_trigger > 0) {
          robot.LaunchMotor.setPower(-1);
      }else {
          robot.LaunchMotor.setPower(0);
      }
/*        if (!gamepad1.right_bumper && !gamepad1.left_bumper) {
            robot.setMotorPower(-DLY, -DRY, -DLY, -DRY);
        }
*/
        telemetry.addData("FL position : ", robot.DriveFrontLeft.getCurrentPosition());
        telemetry.addData("FR Position: ", robot.DriveFrontRight.getCurrentPosition());
        telemetry.addData("BL position : ", robot.DriveBackLeft.getCurrentPosition());
        telemetry.addData("BR Position: ", robot.DriveBackRight.getCurrentPosition());
    }

}
