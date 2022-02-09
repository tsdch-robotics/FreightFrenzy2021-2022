package org.firstinspires.ftc.teamcode.TestFiles;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.SerialNumber;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.Hardware.ChampBot;

@Config
@Autonomous(name="PID-Thermal", group="ChampBot")

public class PIDThermal extends LinearOpMode {


    //private BNO055IMU imu; //model number of inertial measurement unit in REV Expansion hub
    ChampBot robot = new ChampBot();
    public static double kP = 0.3;
    public static double kI = 0.001;
    public static double kD = 0.01;
    PIDControllerHor control = new PIDControllerHor(kP, kI, kD);

    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard;

    ElapsedTime timer = new ElapsedTime();
    double error;
    double target;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        robot.ArmMotorHor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //imu = hardwareMap.get(BNO055IMU.class, (SerialNumber) imu); // added (SerialNumber) as suggested by debugger
        //BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        //parameters.mode = BNO055IMU.SensorMode.IMU;
        //parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        //imu.initialize(parameters);

        dashboard = FtcDashboard.getInstance();

        waitForStart();

        double referenceAngle = Math.toRadians(90);

        while (opModeIsActive()) {
            target = 300;
            error = target- robot.ArmMotorHor.getCurrentPosition();
            while (Math.abs(error) > 10) {
                double command = control.update(target, robot.ArmMotorHor.getCurrentPosition());
                robot.ArmMotorHor.setPower(command);
                error = target - robot.ArmMotorHor.getCurrentPosition();
            }
            robot.ArmMotorHor.setPower(0);
            packet.put("Error ", target);
            packet.put("Current Pos:", robot.ArmMotorHor.getCurrentPosition());
            packet.put("P:", kP);
            packet.put("I", kI);
            packet.put("D", kD);
            dashboard.sendTelemetryPacket(packet);

        }
    }
}