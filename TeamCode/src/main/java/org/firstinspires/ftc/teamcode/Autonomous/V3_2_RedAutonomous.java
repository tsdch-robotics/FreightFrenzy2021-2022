package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.ChampBot;

@Autonomous(name="V3_2_RedAutonomous", group="ChampBot")

public class V3_2_RedAutonomous extends LinearOpMode {
    ChampBot robot = new ChampBot();
    private ElapsedTime runtime = new ElapsedTime();
    static final double tickCount = 537.7;
    static final double wheelDiameter = 3.78; //in inches
    static final double countsPerInch = tickCount/(wheelDiameter*3.1415);
    static final double driveSpeed = 1.0;
    static final double strafeSpeed = 0.5;
    public DcMotor DriveFrontLeft; //:D
    public DcMotor DriveFrontRight;
    public DcMotor DriveBackLeft;
    public DcMotor DriveBackRight;
    public enum Direction {
        left, right;
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

        telemetry.addData("Path0", "Starting at %7d :%7d :%7d :%7d");
        robot.DriveFrontLeft.getCurrentPosition();
        robot.DriveFrontRight.getCurrentPosition();
        robot.DriveBackLeft.getCurrentPosition();
        robot.DriveBackRight.getCurrentPosition();
        telemetry.update();

        waitForStart();




        //first cycle
        //sleep(5000);
        encoderDrive(.4,-7,-7,2.0);
        sleep(100);
        encoderStrafe(.5,17, V3_2_RedAutonomous.Direction.left,2.5); //strafe left
        sleep(100);
        encoderArm(0.7, 2500, 2); //arm up
        encoderArmHor(0.3, 500, 3); //arm to goal
        robot.IntakeMotor.setPower(.3);
        sleep(750);//drop block
        robot.IntakeMotor.setPower(0);
        encoderArmHor(0.3, -450, 3); //arm to center
        sleep(100);
        encoderArm(0.6, -2400, 2); //arm back down

        encoderStrafe(.5,6, V3_2_RedAutonomous.Direction.right,2.0); //strafe right
        encoderStrafe(.2,12, V3_2_RedAutonomous.Direction.right,2.5); //align against wall
        sleep(200);
        encoderDrive(.5,14,14,2.5); //dirve forward a bit
        encoderDriveAndIntake(.2,14,14,-.5,2.5); //drive forward slowly while spinning intake wheel
        sleep(300);
        encoderDrive(-.5,-17,-17,4.0); //drive backwards to original position
        encoderStrafe(.3,12, V3_2_RedAutonomous.Direction.right,2.0);

        //second cycle
        encoderDrive(.3,-7,-7,1.5);
        sleep(100);
        encoderStrafe(.5,17, V3_2_RedAutonomous.Direction.left,2.5); //strafe left
        sleep(100);

        encoderArm(0.7, 2500, 2); //arm up
        encoderArmHor(0.3, 500, 3); //arm to goal
        robot.IntakeMotor.setPower(.3);
        sleep(750);//drop block
        robot.IntakeMotor.setPower(0);
        encoderArmHor(0.3, -500, 3); //arm to center
        sleep(100);
        encoderArm(0.6, -2400, 2); //arm back down

        encoderStrafe(.5,8, V3_2_RedAutonomous.Direction.right,2.5); //strafe right
        encoderStrafe(.2,10, V3_2_RedAutonomous.Direction.right,1.5); //align against wall
        sleep(200);
        encoderDrive(.5,12,12,2.0); //dirve forward a bit
        encoderDriveAndIntake(.2,15,15,-.5,2.0); //drive forward slowly while spinning intake wheel

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

    public void encoderDriveAndIntake (double speed, double leftInches, double rightInches, double intake, double timeoutS) {
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
            robot.IntakeMotor.setPower(intake);

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
            robot.IntakeMotor.setPower(0);

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
            robot.DriveFrontLeft.setPower(strafeSpeed);
            robot.DriveFrontRight.setPower(strafeSpeed);
            robot.DriveBackLeft.setPower(strafeSpeed);
            robot.DriveBackRight.setPower(strafeSpeed);

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
        double inchCount = 4.22 * Math.PI;

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

    //public void setArm (double position) {

    //}
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
}
