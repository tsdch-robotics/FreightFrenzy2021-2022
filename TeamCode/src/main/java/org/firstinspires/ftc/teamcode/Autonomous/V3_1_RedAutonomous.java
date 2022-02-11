package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.ChampBot;

import java.lang.*;

@Autonomous(name="V3_1_RedAutonomous", group="ChampBot")

public class V3_1_RedAutonomous extends LinearOpMode {
    ChampBot robot = new ChampBot();
    private ElapsedTime runtime = new ElapsedTime();
    static final double tickCount = 537.7;
    static final double wheelDiameter = 3.78; //in inches
    static final double countsPerInch = tickCount/(wheelDiameter*3.1415);
    public DcMotor DriveFrontLeft; //:D
    public DcMotor DriveFrontRight;
    public DcMotor DriveBackLeft;
    public DcMotor DriveBackRight;
    public enum Direction {
        left,right;
    }

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addData("Status: ", "Resetting Encoders");
        telemetry.update();

        robot.DriveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.DriveBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.DriveFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.DriveFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.DriveBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.DriveBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d");
        robot.DriveFrontLeft.getCurrentPosition();
        robot.DriveFrontRight.getCurrentPosition();
        robot.DriveBackLeft.getCurrentPosition();
        robot.DriveBackRight.getCurrentPosition();
        telemetry.update();

        waitForStart();


        encoderStrafe(.5,7, Direction.left,3.0); //strafe left
        encoderDrive(.2,-8,-8,3); //back up to duck wheel
        robot.CarouselMotor.setPower(.4);
        sleep(2750);
        robot.CarouselMotor.setPower(0);
        encoderDrive(.3,10,10,3);
        encoderStrafe(.2,10, Direction.right,3.0); //align against wall
        sleep(100);
        encoderDrive(.3,15,15,3.0); //align with wobble
        sleep(100);
        encoderStrafe(.5,13, Direction.left,3.0); //strafe left
        sleep(100);
        encoderArm(0.7, 2500, 2); //arm up
        encoderArmHor(0.3, 500, 3); //arm to goal
        robot.IntakeMotor.setPower(.3);
        sleep(750);//drop block
        robot.IntakeMotor.setPower(0);
        encoderArmHor(0.3, -500, 3); //arm to center
        sleep(100);
        encoderArm(0.6, -2400, 2); //arm back down
        sleep(100);
        encoderStrafe(.5,6, Direction.right,3.0); //strafe right
        encoderStrafe(.2,12, Direction.right,3.0); //align against wall
        encoderDrive(.4,14,14,3.0); //dirve forward a bit
        encoderDriveAndIntake(.2,14,14,3.0); //drive forward slowly while spinning intake wheel
        sleep(200);
        /*
        encoderDrive(-.4,-25,-25,4.0); //drive backwards to original position
        encoderStrafe(.3,20, Direction.right,4.0);

        //second cycle
        encoderStrafe(.5,14, Direction.left,3.0); //strafe left
        sleep(100);
        encoderArm(0.7, 1500, 2); //arm up
        encoderArmHor(0.3, 500, 3); //arm to goal
        robot.IntakeMotor.setPower(.3);
        sleep(750);//drop block
        robot.IntakeMotor.setPower(0);
        encoderArmHor(0.3, -700, 3); //arm to center
        sleep(500);
        encoderArm(0.4, -700, 2); //arm back down

         */

        //repeat


        //encoderDrive(.5, 29, 29, 3.0);
        //encoderTurn(.5, 1, Direction.right, 1.0);
        //encoderArm(.5,800,3.0);
        //encoderDrive(.3, 9, 9, 3.0);
        //robot.Claw.setPosition(1);
        //sleep(500);
        //encoderDrive(.5, -21,-21, 3.0);
        //encoderTurn(.5,1.05,Direction.right,1.0);
        //encoderDrive(.3,26,26,3.0);
        //robot.CarouselMotor2.setPower(-.5);
        //sleep(2500);
        //robot.CarouselMotor2.setPower(0);
        //encoderStrafe(.5,7,Direction.left, 3.0);
        //encoderTurn(.5,1, Direction.left,1.0);
        //encoderDrive(.5,47,47,5.0);
        //encoderStrafe(.5,20, Direction.right,2.0);
        //encoderDrive(.5,30,30,3.0);
        //encoderStrafe(.5,20, Direction.left,2.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
    public void encoderDrive (double speed, double leftInches, double rightInches, double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        if (opModeIsActive()) {
            newFrontLeftTarget = robot.DriveFrontLeft.getCurrentPosition() + (int) (1.25 * leftInches * countsPerInch);
            newFrontRightTarget = robot.DriveFrontRight.getCurrentPosition() + (int) (1.25 * rightInches * countsPerInch);
            newBackLeftTarget = robot.DriveBackLeft.getCurrentPosition() + (int) (1.25 * leftInches * countsPerInch);
            newBackRightTarget = robot.DriveBackRight.getCurrentPosition() + (int) (1.25 * rightInches * countsPerInch);

            robot.DriveFrontLeft.setTargetPosition(newFrontLeftTarget);
            robot.DriveFrontRight.setTargetPosition(newFrontRightTarget);
            robot.DriveBackLeft.setTargetPosition(newBackLeftTarget);
            robot.DriveBackRight.setTargetPosition(newBackRightTarget);

            robot.DriveFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.DriveFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.DriveBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.DriveBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.DriveFrontLeft.setPower(Math.abs(speed));
            robot.DriveFrontRight.setPower(Math.abs(speed));
            robot.DriveBackLeft.setPower(Math.abs(speed));
            robot.DriveBackRight.setPower(Math.abs(speed));

            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (robot.DriveFrontLeft.isBusy() && robot.DriveFrontRight.isBusy() && robot.DriveBackLeft.isBusy() && robot.DriveBackRight.isBusy()))
                ;
            {
                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                //telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", robot.DriveFrontLeft, robot.DriveFrontRight, robot.DriveBackLeft, robot.DriveBackRight);
                telemetry.update();
            }
            robot.DriveFrontLeft.setPower(0);
            robot.DriveFrontRight.setPower(0);
            robot.DriveBackLeft.setPower(0);
            robot.DriveBackRight.setPower(0);

            robot.DriveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.DriveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.DriveBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.DriveBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.DriveFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.DriveFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.DriveBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.DriveBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);
        }
    }

    public void encoderDriveAndIntake (double speed, double leftInches, double rightInches, double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        if (opModeIsActive()) {
            newFrontLeftTarget = robot.DriveFrontLeft.getCurrentPosition() + (int) (1.25 * leftInches * countsPerInch);
            newFrontRightTarget = robot.DriveFrontRight.getCurrentPosition() + (int) (1.25 * rightInches * countsPerInch);
            newBackLeftTarget = robot.DriveBackLeft.getCurrentPosition() + (int) (1.25 * leftInches * countsPerInch);
            newBackRightTarget = robot.DriveBackRight.getCurrentPosition() + (int) (1.25 * rightInches * countsPerInch);

            robot.DriveFrontLeft.setTargetPosition(newFrontLeftTarget);
            robot.DriveFrontRight.setTargetPosition(newFrontRightTarget);
            robot.DriveBackLeft.setTargetPosition(newBackLeftTarget);
            robot.DriveBackRight.setTargetPosition(newBackRightTarget);

            robot.DriveFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.DriveFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.DriveBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.DriveBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.DriveFrontLeft.setPower(Math.abs(speed));
            robot.DriveFrontRight.setPower(Math.abs(speed));
            robot.DriveBackLeft.setPower(Math.abs(speed));
            robot.DriveBackRight.setPower(Math.abs(speed));

            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (robot.DriveFrontLeft.isBusy() && robot.DriveFrontRight.isBusy() && robot.DriveBackLeft.isBusy() && robot.DriveBackRight.isBusy()))
                robot.IntakeMotor.setPower(-.3);
            {
                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                //telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", robot.DriveFrontLeft, robot.DriveFrontRight, robot.DriveBackLeft, robot.DriveBackRight);
                telemetry.update();
            }
            robot.DriveFrontLeft.setPower(0);
            robot.DriveFrontRight.setPower(0);
            robot.DriveBackLeft.setPower(0);
            robot.DriveBackRight.setPower(0);

            robot.DriveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.DriveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.DriveBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.DriveBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.DriveFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.DriveFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.DriveBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.DriveBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //robot.IntakeWheel.setPower(-0.4);

            sleep(250);
        }
    }

    public void encoderStrafe (double strafeSpeed, double inches, Direction direction, double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        if (opModeIsActive()) {
            newFrontLeftTarget = robot.DriveFrontLeft.getCurrentPosition() + (int) (inches * countsPerInch);
            newFrontRightTarget = robot.DriveFrontRight.getCurrentPosition() + (int) (inches * countsPerInch);
            newBackLeftTarget = robot.DriveBackLeft.getCurrentPosition() + (int) (inches * countsPerInch);
            newBackRightTarget = robot.DriveBackRight.getCurrentPosition() + (int) (inches * countsPerInch);

            robot.DriveFrontLeft.setTargetPosition(newFrontLeftTarget);
            robot.DriveFrontRight.setTargetPosition(newFrontRightTarget);
            robot.DriveBackLeft.setTargetPosition(newBackLeftTarget);
            robot.DriveBackRight.setTargetPosition(newBackRightTarget);

            if (direction == Direction.right) {
                robot.DriveFrontLeft.setTargetPosition(newFrontLeftTarget);
                robot.DriveFrontRight.setTargetPosition(-newFrontRightTarget);
                robot.DriveBackLeft.setTargetPosition(-newBackLeftTarget);
                robot.DriveBackRight.setTargetPosition(newBackRightTarget);
            }else {
                robot.DriveFrontLeft.setTargetPosition(-newFrontLeftTarget);
                robot.DriveFrontRight.setTargetPosition(newFrontRightTarget);
                robot.DriveBackLeft.setTargetPosition(newBackLeftTarget);
                robot.DriveBackRight.setTargetPosition(-newBackRightTarget);
            }

            robot.DriveFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.DriveFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.DriveBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.DriveBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.DriveFrontLeft.setPower(Math.abs(strafeSpeed));
            robot.DriveFrontRight.setPower(Math.abs(strafeSpeed));
            robot.DriveBackLeft.setPower(Math.abs(strafeSpeed));
            robot.DriveBackRight.setPower(Math.abs(strafeSpeed));

            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (robot.DriveFrontLeft.isBusy() && robot.DriveFrontRight.isBusy() && robot.DriveBackLeft.isBusy() && robot.DriveBackRight.isBusy()))
                ;
            {
                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                //telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", robot.DriveFrontLeft, robot.DriveFrontRight, robot.DriveBackLeft, robot.DriveBackRight);
                telemetry.update();
            }
            robot.DriveFrontLeft.setPower(0);
            robot.DriveFrontRight.setPower(0);
            robot.DriveBackLeft.setPower(0);
            robot.DriveBackRight.setPower(0);

            robot.DriveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.DriveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.DriveBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.DriveBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.DriveFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.DriveFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.DriveBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.DriveBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);
        }
    }

    public void encoderTurn (double speed, double numberOfTurns,   Direction direction, double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;
        double inchCount = 6.53 * Math.PI;

        if (opModeIsActive()) {
            newFrontLeftTarget = robot.DriveFrontLeft.getCurrentPosition() + (int) (inchCount * countsPerInch * numberOfTurns);
            newFrontRightTarget = robot.DriveFrontRight.getCurrentPosition() + (int) (inchCount * countsPerInch * numberOfTurns);
            newBackLeftTarget = robot.DriveBackLeft.getCurrentPosition() + (int) (inchCount * countsPerInch * numberOfTurns);
            newBackRightTarget = robot.DriveBackRight.getCurrentPosition() + (int) (inchCount * countsPerInch * numberOfTurns);

            if(direction == Direction.right) {
                robot.DriveFrontLeft.setTargetPosition(newFrontLeftTarget);
                robot.DriveFrontRight.setTargetPosition(-newFrontRightTarget);
                robot.DriveBackLeft.setTargetPosition(newBackLeftTarget);
                robot.DriveBackRight.setTargetPosition(-newBackRightTarget);
            } else {
                robot.DriveFrontLeft.setTargetPosition(-newFrontLeftTarget);
                robot.DriveFrontRight.setTargetPosition(newFrontRightTarget);
                robot.DriveBackLeft.setTargetPosition(-newBackLeftTarget);
                robot.DriveBackRight.setTargetPosition(newBackRightTarget);
            }


            robot.DriveFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.DriveFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.DriveBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.DriveBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.DriveFrontLeft.setPower(Math.abs(speed));
            robot.DriveFrontRight.setPower(Math.abs(speed));
            robot.DriveBackLeft.setPower(Math.abs(speed));
            robot.DriveBackRight.setPower(Math.abs(speed));

            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (robot.DriveFrontLeft.isBusy() && robot.DriveFrontRight.isBusy() && robot.DriveBackLeft.isBusy() && robot.DriveBackRight.isBusy()))
                ;
            {
                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                //telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", robot.DriveFrontLeft, robot.DriveFrontRight, robot.DriveBackLeft, robot.DriveBackRight);
                telemetry.update();
            }
            robot.DriveFrontLeft.setPower(0);
            robot.DriveFrontRight.setPower(0);
            robot.DriveBackLeft.setPower(0);
            robot.DriveBackRight.setPower(0);

            robot.DriveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.DriveFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.DriveBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.DriveBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.DriveFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.DriveFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.DriveBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.DriveBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);
        }
    }

    public void encoderArm (double speed, double armTickCount, double timeoutS) {
        int newArmTarget;

        if (opModeIsActive()) {
            newArmTarget = robot.ArmMotorVert.getCurrentPosition() + (int) (armTickCount);

            robot.ArmMotorVert.setTargetPosition(newArmTarget);

            robot.ArmMotorVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.ArmMotorVert.setPower(Math.abs(speed));

            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (robot.ArmMotorVert.isBusy()))
                ;
            {
                telemetry.addData("Path1", "Running to %7d", newArmTarget);
                //telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", robot.DriveFrontLeft, robot.DriveFrontRight, robot.DriveBackLeft, robot.DriveBackRight);
                telemetry.update();
            }
            robot.ArmMotorVert.setPower(0);

            robot.ArmMotorVert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.ArmMotorVert.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);
        }
    }

    public void encoderArmHor (double speed, double armTickCount, double timeoutS) {
        int newArmTarget;

        if (opModeIsActive()) {
            newArmTarget = robot.ArmMotorHor.getCurrentPosition() + (int) (armTickCount);

            robot.ArmMotorHor.setTargetPosition(newArmTarget);

            robot.ArmMotorHor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.ArmMotorHor.setPower(Math.abs(speed));

            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (robot.ArmMotorHor.isBusy()))
                ;
            {
                telemetry.addData("Path1", "Running to %7d", newArmTarget);
                //telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", robot.DriveFrontLeft, robot.DriveFrontRight, robot.DriveBackLeft, robot.DriveBackRight);
                telemetry.update();
            }
            robot.ArmMotorHor.setPower(0);

            robot.ArmMotorHor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.ArmMotorHor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);
        }
    }
    /*
    public void encoderArm (double speed, double armTickCount, double timeoutS) {
        int newArmTarget;

        if (opModeIsActive()) {
            newArmTarget = robot.ArmMotor.getCurrentPosition() + (int) (armTickCount);

            robot.ArmMotor.setTargetPosition(newArmTarget);

            robot.ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.ArmMotor.setPower(Math.abs(speed));

            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (robot.ArmMotor.isBusy()))
                ;
            {
                telemetry.addData("Path1", "Running to %7d", newArmTarget);
                //telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", robot.DriveFrontLeft, robot.DriveFrontRight, robot.DriveBackLeft, robot.DriveBackRight);
                telemetry.update();
            }
            robot.ArmMotor.setPower(0);

            robot.ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);
        }
    }

     */
}
