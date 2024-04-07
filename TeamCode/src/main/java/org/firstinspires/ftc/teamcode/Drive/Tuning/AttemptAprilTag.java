package org.firstinspires.ftc.teamcode.Drive.Tuning;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.Utility.RoadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
@TeleOp
public class AttemptAprilTag extends LinearOpMode {
    SampleMecanumDrive roadrunnerDrive;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    public Vector2d cameraOffset = new Vector2d(
            -6,
            4);
    @Override
    public void runOpMode() {
        roadrunnerDrive = new SampleMecanumDrive(hardwareMap);
        initAprilTag();

        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetryAprilTag();

                telemetry.update();

                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }

                sleep(20);
            }
        }
        visionPortal.close();

    }

    private void initAprilTag() {

        aprilTag = new AprilTagProcessor.Builder()
            //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506) //TODO: tune this
            .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
            builder.setCameraResolution(new Size(1280, 800));
            builder.addProcessor(aprilTag);
            builder.build();
    }

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                double x = -detection.ftcPose.x-cameraOffset.getX();
                double y = -detection.ftcPose.y-cameraOffset.getY();

                double botHeading = -roadrunnerDrive.getPoseEstimate().getHeading();


                double x2 = x * Math.cos(botHeading) + y * Math.sin(botHeading);
                double y2 = x * -Math.sin(botHeading) + y * Math.cos(botHeading);
                double absX;
                double absY;

                VectorF tagPosition = getCenterStageTagLibrary().lookupTag(detection.id).fieldPosition;
                if (detection.metadata.id <= 6) {
                    absX = tagPosition.get(0) + y2;
                    absY = tagPosition.get(1) - x2;

                } else {
                    absX = tagPosition.get(0) - y2;
                    absY = tagPosition.get(1) + x2;

                }

                Pose2d fieldCentricPosition = new Pose2d(absX, absY, botHeading);
                telemetry.addData("Position: ", fieldCentricPosition);
                roadrunnerDrive.setPoseEstimate(fieldCentricPosition);
            }
        }

        roadrunnerDrive.update();

    }
    public static AprilTagLibrary getCenterStageTagLibrary()
    {
        return new AprilTagLibrary.Builder()
                .addTag(1, "BlueAllianceLeft",
                        2, new VectorF(61.75f, 41.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(2, "BlueAllianceCenter",
                        2, new VectorF(61.75f, 35.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(3, "BlueAllianceRight",
                        2, new VectorF(61.75f, 29.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(4, "RedAllianceLeft",
                        2, new VectorF(61.75f, -29.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(5, "RedAllianceCenter",
                        2, new VectorF(61.75f, -35.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(6, "RedAllianceRight",
                        2, new VectorF(61.75f, -41.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(7, "RedAudienceWallLarge",
                        5, new VectorF(-70.25f, -40.625f, 5.5f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .addTag(8, "RedAudienceWallSmall",
                        2, new VectorF(-70.25f, -35.125f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .addTag(9, "BlueAudienceWallSmall",
                        2, new VectorF(-70.25f, 35.125f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .addTag(10, "BlueAudienceWallLarge",
                        5, new VectorF(-70.25f, 40.625f, 5.5f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .build();
    }
}

