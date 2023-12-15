package org.firstinspires.ftc.teamcode.drive.opmode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.common.commandbase.command.TapeDrop;
import org.firstinspires.ftc.teamcode.common.commandbase.command.OuttakePosition;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.Drive;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous
public class BlueLeft extends OpMode {
    private Robot robot;
    private ElapsedTime time_since_start;
    TfodProcessor myTfodProcessor;
    int elementPosition;
    boolean USE_WEBCAM;
    private double loop;
    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        robot = new Robot(hardwareMap);

        CommandScheduler.getInstance().registerSubsystem(robot.a);
        CommandScheduler.getInstance().registerSubsystem(robot.claw);
        CommandScheduler.getInstance().registerSubsystem(robot.angle);
        CommandScheduler.getInstance().registerSubsystem(robot.drive);

        USE_WEBCAM = true;
        initTfod();
        telemetry.addData("D.S. Preview: ", "Ready for BlueLeft (Backdrop Side)");
        telemetry.update();

        robot.claw.grabBoth();

    }

    @Override
    public void init_loop() {
        telemetryTfod();
        CommandScheduler.getInstance().run();
        robot.a.loop();
    }

    public void start() {
        time_since_start = new ElapsedTime();
        if(elementPosition == 0) { //right

            TrajectorySequence dropPixelRight = robot.drive.trajectorySequenceBuilder(new Pose2d(16.11, 63.82, Math.toRadians(-90.00)))
                    .lineToSplineHeading(new Pose2d(11.93, 39.96, Math.toRadians(30.00)))
                    .build();

            TrajectorySequence backdropPixelRight = robot.drive.trajectorySequenceBuilder(dropPixelRight.end())
                    .splineTo(new Vector2d(13.49, 39.79), Math.toRadians(17.35))
                    .build();

            TrajectorySequence preParkRight = robot.drive.trajectorySequenceBuilder(backdropPixelRight.end())
                    .splineTo(new Vector2d(48.15, 28.82), Math.toRadians(0.00))
                    .build();

            TrajectorySequence parkRight = robot.drive.trajectorySequenceBuilder(preParkRight.end())
                    .lineToConstantHeading(new Vector2d(48.67, 62.95))
                    .lineToConstantHeading(new Vector2d(63.47, 63.12))
                    .build();

            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new WaitCommand(1000),
                            new TapeDrop(robot),
                            new InstantCommand(() -> robot.drive.followTrajectorySequence(dropPixelRight)),
                            new WaitCommand(350),
                            new InstantCommand(() -> robot.claw.releaseLeft()),
                            new WaitCommand(350),
                            new ParallelCommandGroup(
                                    new InstantCommand(() -> robot.claw.grabLeft()),
                                    new OuttakePosition(robot),
                                    new InstantCommand(() -> robot.drive.followTrajectorySequence(backdropPixelRight))
                            ),
                            new WaitCommand(350),
                            new InstantCommand(() -> robot.claw.releaseRight()),
                            new WaitCommand(350),
                            new ParallelCommandGroup(
                                    new InstantCommand(() -> robot.claw.grabRight()),
                                    new InstantCommand(() -> robot.a.armCoast())
                            ),
                            new InstantCommand(() -> robot.drive.followTrajectorySequence(preParkRight)),
                            new InstantCommand(() -> robot.drive.followTrajectorySequence(parkRight))
                    )
            );
        } else if(elementPosition == 1) { //middle
            TrajectorySequence dropPixelMiddle = robot.drive.trajectorySequenceBuilder(new Pose2d(18.89, 66.78, Math.toRadians(-90.00)))
                    .lineToConstantHeading(
                            new Vector2d(15.06, 25.51),
                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .build();

            TrajectorySequence backdropPixelMiddle = robot.drive.trajectorySequenceBuilder(dropPixelMiddle.end())
                    .lineToConstantHeading(
                            new Vector2d(15.41, 45.71),
                            SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .splineTo(
                            new Vector2d(47.80, 37.87), Math.toRadians(0.00),
                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .build();

            TrajectorySequence parkMiddle = robot.drive.trajectorySequenceBuilder(backdropPixelMiddle.end())
                    .lineToConstantHeading(
                            new Vector2d(62.07, 60.51),
                            SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .lineToConstantHeading(
                            new Vector2d(48.15, 60.68),
                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .build();


            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new WaitCommand(500),
                            new InstantCommand(() -> robot.drive.followTrajectorySequence(dropPixelMiddle)),
                            new WaitCommand(1000),
                            new ParallelCommandGroup(
                                    new OuttakePosition(robot),
                                    new InstantCommand(() -> robot.drive.followTrajectorySequence(backdropPixelMiddle))
                            )
//                            new WaitCommand(1000),
//                            new InstantCommand(() -> robot.claw.releaseRight()),
//                            new WaitCommand(1000),
//                            new ParallelCommandGroup(
//                                    new InstantCommand(() -> robot.claw.grabRight()),
//                                    new InstantCommand(() -> robot.a.armCoast())
//                            ),
//                            new InstantCommand(() -> robot.drive.followTrajectorySequence(preParkMiddle)),
//                            new InstantCommand(() -> robot.drive.followTrajectorySequence(parkMiddle))
                    )
            );

        } else if(elementPosition == 1) { //left
            TrajectorySequence dropPixelLeft = robot.drive.trajectorySequenceBuilder(new Pose2d(16.11, 63.82, Math.toRadians(-90.00)))
                    .lineToSplineHeading(new Pose2d(23.94, 47.10, Math.toRadians(90.00)))
                    .build();

            TrajectorySequence backdropPixelLeft = robot.drive.trajectorySequenceBuilder(dropPixelLeft.end())
                    .splineTo(new Vector2d(48.84, 41.88), Math.toRadians(0.00))
                    .build();

            TrajectorySequence preParkLeft = robot.drive.trajectorySequenceBuilder(backdropPixelLeft.end())
                    .lineToConstantHeading(new Vector2d(48.67, 62.95))
                    .build();

            TrajectorySequence parkLeft = robot.drive.trajectorySequenceBuilder(preParkLeft.end())
                    .lineToConstantHeading(new Vector2d(63.47, 63.12))
                    .build();

            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new WaitCommand(1000),
                            new TapeDrop(robot),
                            new InstantCommand(() -> robot.drive.followTrajectorySequence(dropPixelLeft)),
                            new WaitCommand(350),
                            new InstantCommand(() -> robot.claw.releaseLeft()),
                            new WaitCommand(350),
                            new ParallelCommandGroup(
                                    new InstantCommand(() -> robot.claw.grabLeft()),
                                    new OuttakePosition(robot),
                                    new InstantCommand(() -> robot.drive.followTrajectorySequence(backdropPixelLeft))
                            ),
                            new WaitCommand(1000),
                            new InstantCommand(() -> robot.claw.releaseRight()),
                            new WaitCommand(1000),
                            new ParallelCommandGroup(
                                    new InstantCommand(() -> robot.claw.grabRight()),
                                    new InstantCommand(() -> robot.a.armCoast())
                            ),
                            new InstantCommand(() -> robot.drive.followTrajectorySequence(preParkLeft)),
                            new InstantCommand(() -> robot.drive.followTrajectorySequence(parkLeft))
                    )
            );
        }
    }
    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        robot.a.loop();
        robot.drive.update();

        double time = System.currentTimeMillis();
        telemetry.addData("Loop: ", time - loop);
        telemetry.addData("Arm Position: ", robot.a.getCachePos());
        loop = time;

        telemetry.update();
    }

    @Override
    public void stop() {
        CommandScheduler.getInstance().reset();
    }

    private void initTfod() {
        TfodProcessor.Builder myTfodProcessorBuilder;
        VisionPortal.Builder myVisionPortalBuilder;
        VisionPortal myVisionPortal;

        myTfodProcessorBuilder = new TfodProcessor.Builder();
        myTfodProcessorBuilder.setModelFileName("BLUETURKEY.tflite");
        myTfodProcessorBuilder.setModelLabels(JavaUtil.createListWith("TSE"));
        myTfodProcessorBuilder.setModelAspectRatio(16 / 9);
        myTfodProcessor = myTfodProcessorBuilder.build();
        myVisionPortalBuilder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam"));
        } else {
            myVisionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
        }
        myVisionPortalBuilder.addProcessor(myTfodProcessor);
        myVisionPortal = myVisionPortalBuilder.build();
    }
    private void telemetryTfod() {
        boolean sensedElement;
        List<Recognition> myTfodRecognitions;
        Recognition myTfodRecognition;
        float x;
        float y;

        sensedElement = false;
        myTfodRecognitions = (List<Recognition>) myTfodProcessor.getRecognitions();
        telemetry.addData("# Objects Detected", JavaUtil.listLength(myTfodRecognitions));
        for (Recognition myTfodRecognition_item : myTfodRecognitions) {
            myTfodRecognition = myTfodRecognition_item;
            if (myTfodRecognition.getConfidence() * 100 > 70) {
                telemetry.addLine("");
                telemetry.addData("Image", myTfodRecognition.getLabel() + " (" + JavaUtil.formatNumber(myTfodRecognition.getConfidence() * 100, 0) + " % Conf.)");
                x = (myTfodRecognition.getLeft() + myTfodRecognition.getRight()) / 2;
                y = (myTfodRecognition.getTop() + myTfodRecognition.getBottom()) / 2;
                telemetry.addData("- Position", JavaUtil.formatNumber(x, 0) + ", " + JavaUtil.formatNumber(y, 0));
                telemetry.addData("- Size", JavaUtil.formatNumber(myTfodRecognition.getWidth(), 0) + " x " + JavaUtil.formatNumber(myTfodRecognition.getHeight(), 0));
                if (x > 500) {
                    sensedElement = true;
                    elementPosition = 0;
                    telemetry.addData("Team Element Detection: ", "Right");
                } else if (x >= 190 && x <= 250) {
                    sensedElement = true;
                    elementPosition = 1;
                    telemetry.addData("Team Element Detection: ", "Middle");
                }
            }
        }
        if (!sensedElement) {
            elementPosition = 2;
            telemetry.addData("Team Element Detection: ", "Left");
        }
    }
    //0: right
    //1: middle
    //2: left
}
