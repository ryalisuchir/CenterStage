package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous
public class BlueLeft extends LinearOpMode {

    SampleMecanumDrive drive;
    TfodProcessor myTfodProcessor;
    int turkeyPos;
    boolean USE_WEBCAM;
    boolean senseYeah;

    private PIDController controller;
    public static double p = 0.01, i = 0, d = 0;
    public static double f = 0.1;
    public static int target = 0;
    private final double ticks_in_degree = 384.5 / 180; //depends on motor you use
    private DcMotorEx linear_1;
    private DcMotorEx linear_2;

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() throws InterruptedException {

        // This 2023-2024 OpMode illustrates the basics of TensorFlow Object Detection, using
        // a custom TFLite object detection model.
        USE_WEBCAM = true;
        // Initialize TFOD before waitForStart.
        initTfod();
        // Wait for the match to begin.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        drive = new SampleMecanumDrive(hardwareMap);
        while (!opModeIsActive()) {
            // Put loop blocks here.
            telemetryTfod();
            // Push telemetry to the Driver Station.
            telemetry.update();
        }
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        linear_1 = hardwareMap.get(DcMotorEx.class, "linear_1");
        linear_2 = hardwareMap.get(DcMotorEx.class, "linear_2");
        linear_2.setDirection(DcMotor.Direction.REVERSE);
        linear_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linear_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        if (opModeIsActive()) {
            // Put run blocks here.
            //nothing=right
            //1=middle
            //2=left
            if (turkeyPos == 1) { //middle
                TrajectorySequence middle = drive.trajectorySequenceBuilder(new Pose2d(15.63, 64.10, Math.toRadians(270.00)))
                        .splineTo(
                                new Vector2d(6.67, 37.05), Math.toRadians(212.47),
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToConstantHeading(new Vector2d(18.97, 50.58))
                        .splineTo(
                                new Vector2d(50.58, 59.88), Math.toRadians(0.00),
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                drive.followTrajectorySequence(middle);

            } else if (turkeyPos == 2) { //left
                TrajectorySequence left = drive.trajectorySequenceBuilder(new Pose2d(15.63, 64.10, Math.toRadians(270.00)))
                        .splineToConstantHeading(
                                new Vector2d(33.72, 42.15), Math.toRadians(270.00),
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineTo(new Vector2d(35.65, 53.56))
                        .splineTo(
                                new Vector2d(59.18, 54.61), Math.toRadians(0.00),
                                SampleMecanumDrive.getVelocityConstraint(23, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();



                drive.followTrajectorySequence(left);

            } else if (!senseYeah){ //right
                TrajectorySequence right = drive.trajectorySequenceBuilder(new Pose2d(15.28, 64.80, Math.toRadians(270.00)))
                        .splineTo(
                                new Vector2d(5.80, 36.00), Math.toRadians(212.47),
                                SampleMecanumDrive.getVelocityConstraint(23, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToConstantHeading(new Vector2d(18.61, 51.10))
                        .splineTo(
                                new Vector2d(50.40, 57.60), Math.toRadians(0.00),
                                SampleMecanumDrive.getVelocityConstraint(23, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                drive.followTrajectorySequence(right);


            }
        }

    }

    /**
     * Initialize TensorFlow Object Detection.
     */
    private void initTfod() {
        TfodProcessor.Builder myTfodProcessorBuilder;
        VisionPortal.Builder myVisionPortalBuilder;
        VisionPortal myVisionPortal;

        // First, create a TfodProcessor.Builder.
        myTfodProcessorBuilder = new TfodProcessor.Builder();
        // Set the name of the file where the model can be found.
        myTfodProcessorBuilder.setModelFileName("BLUETURKEY.tflite");
        // Set the full ordered list of labels the model is trained to recognize.
        myTfodProcessorBuilder.setModelLabels(JavaUtil.createListWith("TSE"));
        // Set the aspect ratio for the images used when the model was created.
        myTfodProcessorBuilder.setModelAspectRatio(16 / 9);
        // Create a TfodProcessor by calling build.
        myTfodProcessor = myTfodProcessorBuilder.build();
        // Next, create a VisionPortal.Builder and set attributes related to the camera.
        myVisionPortalBuilder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            // Use a webcam.
            myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam"));
        } else {
            // Use the device's back camera.
            myVisionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
        }
        // Add myTfodProcessor to the VisionPortal.Builder.
        myVisionPortalBuilder.addProcessor(myTfodProcessor);
        // Create a VisionPortal by calling build.
        myVisionPortal = myVisionPortalBuilder.build();
    }

    /**
     * Display info (using telemetry) for a detected object
     */
    private void telemetryTfod() {
        boolean senseYeah;
        List<Recognition> myTfodRecognitions;
        Recognition myTfodRecognition;
        float x;
        float y;

        senseYeah = false;
        // Get a list of recognitions from TFOD.
        myTfodRecognitions = myTfodProcessor.getRecognitions();
        telemetry.addData("# Objects Detected", JavaUtil.listLength(myTfodRecognitions));
        // Iterate through list and call a function to display info for each recognized object.
        for (Recognition myTfodRecognition_item : myTfodRecognitions) {
            myTfodRecognition = myTfodRecognition_item;
            // minimum confidence level
            if (myTfodRecognition.getConfidence() * 100 > 70) {
                // Display info about the recognition.
                telemetry.addLine("");
                // Display label and confidence.
                // Display the label and confidence for the recognition.
                telemetry.addData("Image", myTfodRecognition.getLabel() + " (" + JavaUtil.formatNumber(myTfodRecognition.getConfidence() * 100, 0) + " % Conf.)");
                // Display position.
                x = (myTfodRecognition.getLeft() + myTfodRecognition.getRight()) / 2;
                y = (myTfodRecognition.getTop() + myTfodRecognition.getBottom()) / 2;
                // Display the position of the center of the detection boundary for the recognition
                telemetry.addData("- Position", JavaUtil.formatNumber(x, 0) + ", " + JavaUtil.formatNumber(y, 0));
                // Display size
                // Display the size of detection boundary for the recognition
                telemetry.addData("- Size", JavaUtil.formatNumber(myTfodRecognition.getWidth(), 0) + " x " + JavaUtil.formatNumber(myTfodRecognition.getHeight(), 0));
                if (x > 500) {
                    senseYeah = true;
                    turkeyPos = 0;
                    telemetry.addData("TurkeyPos", "Right");
                } else if (x >= 190 && x <= 250) {
                    senseYeah = true;
                    turkeyPos = 1;
                    telemetry.addData("TurkeyPos", "Middle");
                }
            }
        }
        if (!senseYeah) {
            turkeyPos = 2;
            telemetry.addData("TurkeyPos", "Left");
        }
    }
}