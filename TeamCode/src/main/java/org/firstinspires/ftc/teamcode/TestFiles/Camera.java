package org.firstinspires.ftc.teamcode.TestFiles;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Hardware.ChampBot_v2;

@TeleOp(name = "Concept: TensorFlow Object Detection", group = "Concept")

public class Camera extends LinearOpMode {

    ChampBot_v2 robot = new ChampBot_v2();
    private ElapsedTime runtime = new ElapsedTime();

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AT3AS3b/////AAABmeb4lXVRG0y7kvB4fG/FtbKNEuekc1o6rkU8fj5JsakaaPPNIV+ZalkosfH2Zl513dKhMTyUF6YtHzQUr07sQDtKs2GbaulwZFC5yov+tEsgEHA+OWVni0f18T87qrboMBnj61MUqZP4BvjPib2R1rxEr3Cj4YS9c9RnkUESyTBI6B/gCtBFLNKh7cX9PKJnKCgkVF9qexL2eu2lRS1iNAl9VsjBel6RSckVjGEo90+DSyVMgUtew2GNk8IokXYlBHlK7MY6ju3Kb1dFyLLm0RPcXxqvA3pIPNbk8Rsuv75xhBGBRMVpgteBaaWZz6YUowvEv5YfGdTjy1kmQyzufcwqvMY038k0BzPCUGXE77Md";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            tfod.setZoom(1.0, 1.77777);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                /*
                double FrontLeftPower;
                double FrontRightPower;
                double RearLeftPower;
                double RearRightPower;

                FrontLeftDrive.setPower(FrontLeftPower);
                FrontRightDrive.setPower(FrontRightPower);
                RearLeftDrive.setPower(RearLeftPower);
                RearRightDrive.setPower(RearRightPower);
                */

                if (tfod != null) {



                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());

                        int i = 0;

                        for (Recognition recognition : updatedRecognitions) {

                            if (recognition.getConfidence() >= 0.7) {
                                if (i==0) {
                                    if (recognition.estimateAngleToObject(AngleUnit.DEGREES) >= 10) {
                                        robot.DriveFrontLeft.setPower(-0.2);
                                        robot.DriveFrontRight.setPower(0.2);
                                        robot.DriveBackLeft.setPower(-0.2);
                                        robot.DriveBackRight.setPower(0.2);
                                    } else if (recognition.estimateAngleToObject(AngleUnit.DEGREES) <= -10) {
                                        robot.DriveFrontLeft.setPower(0.2);
                                        robot.DriveFrontRight.setPower(-0.2);
                                        robot.DriveBackLeft.setPower(0.2);
                                        robot.DriveBackRight.setPower(-0.2);
                                    } else {
                                        robot.DriveFrontLeft.setPower(0);
                                        robot.DriveFrontRight.setPower(0);
                                        robot.DriveBackLeft.setPower(0);
                                        robot.DriveBackRight.setPower(0);
                                    }
                                }
                            } else {
                                robot.DriveFrontLeft.setPower(0);
                                robot.DriveFrontRight.setPower(0);
                                robot.DriveBackLeft.setPower(0);
                                robot.DriveBackRight.setPower(0);
                            }

                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData("Confidence: ", "%f", recognition.getConfidence());


                        }
                        telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.6f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
