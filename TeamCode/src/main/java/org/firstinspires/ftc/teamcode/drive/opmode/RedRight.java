package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.teamcode.util.ColourMassDetectionProcessor.PropPositions.LEFT;
import static org.firstinspires.ftc.teamcode.util.ColourMassDetectionProcessor.PropPositions.MIDDLE;
import static org.firstinspires.ftc.teamcode.util.ColourMassDetectionProcessor.PropPositions.RIGHT;
import static org.firstinspires.ftc.teamcode.util.ColourMassDetectionProcessor.PropPositions.UNFOUND;

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
import org.firstinspires.ftc.teamcode.util.ColourMassDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.opencv.core.Scalar;

@Autonomous
public class RedRight extends LinearOpMode {
    private VisionPortal visionPortal;
    private ColourMassDetectionProcessor colourMassDetectionProcessor;
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
        // Wait for the match to begin.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        drive = new SampleMecanumDrive(hardwareMap);
        //Scalar lower = new Scalar(80, 50, 50);         // lower color border for BLUE
        //Scalar upper = new Scalar(150, 255, 225);      // upper color border for BLUE
        Scalar lower = new Scalar(150, 100, 100); // the lower hsv threshold
        Scalar upper = new Scalar(180, 255, 255); // the upper hsv threshold
        double minArea = 100; // the minimum area for the detection to consider

        colourMassDetectionProcessor = new ColourMassDetectionProcessor(
                lower,
                upper,
                () -> minArea, // these are lambda methods, in case we want to change them while the match is running, for us to tune them or something
                () -> 213, // left third
                () -> 426 // right third
        );
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .addProcessor(colourMassDetectionProcessor)
                .build();
        while (!opModeIsActive()) {
            telemetry.addData("Currently Recorded Position: ", colourMassDetectionProcessor.getRecordedPropPosition());
            telemetry.addData("Camera State: ", visionPortal.getCameraState());
            // Put loop blocks here.
            // Push telemetry to the Driver Station.
            telemetry.update();
        }
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.stopLiveView();
            visionPortal.stopStreaming();
        }

        ColourMassDetectionProcessor.PropPositions recordedPropPosition = colourMassDetectionProcessor.getRecordedPropPosition();

        if (recordedPropPosition == UNFOUND) {
            recordedPropPosition = ColourMassDetectionProcessor.PropPositions.MIDDLE;
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
        if (recordedPropPosition == LEFT || recordedPropPosition == RIGHT || recordedPropPosition == UNFOUND || recordedPropPosition == MIDDLE) { //left and left
                TrajectorySequence left = drive.trajectorySequenceBuilder(new Pose2d(10.89, -67.43, Math.toRadians(90.00)))
                        .splineTo(
                                new Vector2d(6.32, -40.21), Math.toRadians(124.59),
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToConstantHeading(new Vector2d(19.32, -52.68))
                        .lineToSplineHeading(
                                new Pose2d(73.40, -69.72, Math.toRadians(0.00)),
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();


                drive.followTrajectorySequence(left);

            };
//            else if (recordedPropPosition == RIGHT) { //right
//                TrajectorySequence right = drive.trajectorySequenceBuilder(new Pose2d(8.78, -70.07, Math.toRadians(90.00)))
//                        .lineToConstantHeading(new Vector2d(22.65, -35.12))
//                        .lineTo(new Vector2d(22.83, -46.54))
//                        .lineToSplineHeading(new Pose2d(58.83, -61.64, Math.toRadians(0.00)))
//                        .build();


                //drive.followTrajectorySequence(right);


            }
        }

    }

    /**
     * Initialize TensorFlow Object Detection.
     */