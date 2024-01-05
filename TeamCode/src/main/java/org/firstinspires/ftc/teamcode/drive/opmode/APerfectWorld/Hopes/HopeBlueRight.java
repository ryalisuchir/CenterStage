package org.firstinspires.ftc.teamcode.drive.opmode.APerfectWorld.Hopes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.commandbase.command.GrabBothCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.RestCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.TapeDropperCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.commandbase.command.OuttakeCommand;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.ColorPropDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

@Autonomous
@Config
public class HopeBlueRight extends OpMode {
    private VisionPortal visionPortal;
    private ColorPropDetectionProcessor colorMassDetectionProcessor;

    private Robot robot;
    private ElapsedTime time_since_start;
    private double loop;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        robot = new Robot(hardwareMap);

        CommandScheduler.getInstance().registerSubsystem(robot.a);
        CommandScheduler.getInstance().registerSubsystem(robot.claw);
        CommandScheduler.getInstance().registerSubsystem(robot.angle);
        CommandScheduler.getInstance().registerSubsystem(robot.driveSubsystem);

        telemetry.addData("Not Ready: ", "Not able to proceed to camera detection... Restart robot now.");
        telemetry.update();

        robot.claw.grabBoth();

        Scalar lower = new Scalar(80, 50, 50);
        Scalar upper = new Scalar(180, 255, 255);
        double minArea = 100;

        colorMassDetectionProcessor = new ColorPropDetectionProcessor(
                lower,
                upper,
                () -> minArea,
                () -> 213, //left third of frame
                () -> 426 //right third of frame
        );
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .addProcessor(colorMassDetectionProcessor)
                .build();
    }

    @Override
    public void init_loop() {
        telemetry.addData("Successful: ", "Ready for BlueRight (Not Backdrop Side)");
        telemetry.addData("Ready to Run: ", "2 pixel autonomous. All subsystems initialized.");
        telemetry.addData("Currently Recorded Position", colorMassDetectionProcessor.getRecordedPropPosition());
        telemetry.addData("Camera State", visionPortal.getCameraState());
        telemetry.addData("Currently Detected Mass Center", "x: " + colorMassDetectionProcessor.getLargestContourX() + ", y: " + colorMassDetectionProcessor.getLargestContourY());
        telemetry.addData("Currently Detected Mass Area", colorMassDetectionProcessor.getLargestContourArea());
        CommandScheduler.getInstance().run();
        robot.a.loop();
    }

    @Override
    public void start() {
        time_since_start = new ElapsedTime();
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.stopLiveView();
            visionPortal.stopStreaming();
        }

        ColorPropDetectionProcessor.PropPositions recordedPropPosition = colorMassDetectionProcessor.getRecordedPropPosition();
        robot.driveSubsystem.setPoseEstimate(new Pose2d(-39.61, 68.34, Math.toRadians(270.00)));
        switch (recordedPropPosition) {
            case LEFT:
            case UNFOUND:
                TrajectorySequence tapeLeft = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-39.61, 68.34, Math.toRadians(270.00)))
                        .splineTo(new Vector2d(-32.30, 38.39), Math.toRadians(-51.34))
                        .build();

                TrajectorySequence initialBackdropLeft = robot.driveSubsystem.trajectorySequenceBuilder(tapeLeft.end())
                        .lineToConstantHeading(new Vector2d(-45.01, 53.89))
                        .lineToSplineHeading(new Pose2d(-44.49, 11.58, Math.toRadians(270.00)))
                        .lineToSplineHeading(new Pose2d(48.84, 12.80, Math.toRadians(0.00)))
                        .build();

                TrajectorySequence lastBackdropLeft = robot.driveSubsystem.trajectorySequenceBuilder(initialBackdropLeft.end())
                        .lineToConstantHeading(new Vector2d(48.67, 42.57))
                        .build();

                TrajectorySequence parkLeft = robot.driveSubsystem.trajectorySequenceBuilder(lastBackdropLeft.end())
                        .lineToConstantHeading(new Vector2d(40.83, 42.75))
                        .lineToConstantHeading(new Vector2d(39.44, 13.15))
                        .splineTo(new Vector2d(66.25, 12.45), Math.toRadians(0.00))
                        .build();

                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequencenotAsync(tapeLeft)),
                                new WaitCommand(500),
                                new ParallelCommandGroup(
                                        new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequencenotAsync(initialBackdropLeft)),
                                        new RestCommand(robot)
                                ),
                                new WaitCommand(500),
                                new OuttakeCommand(robot),
                                new WaitCommand(0), //edit this to wait before moving - track the time here if needed to find max
                                new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequencenotAsync(lastBackdropLeft)),
                                new WaitCommand(1000),
                                new InstantCommand(() -> robot.claw.autoReleaseLeft()),
                                new WaitCommand(1000),
                                new ParallelCommandGroup(
                                        new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequencenotAsync(parkLeft)),
                                        new GrabBothCommand(robot),
                                        new RestCommand(robot)
                                )

                        )
                );
                break;
            case RIGHT:
                TrajectorySequence tapeRight = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-39.61, 68.34, Math.toRadians(270.00)))
                        .splineToConstantHeading(new Vector2d(-46.23, 40.14), Math.toRadians(270.00))
                        .build();

                TrajectorySequence initialBackdropRight = robot.driveSubsystem.trajectorySequenceBuilder(tapeRight.end())
                        .lineToConstantHeading(new Vector2d(-46.40, 52.50))
                        .lineToConstantHeading(new Vector2d(-35.26, 51.98))
                        .lineToSplineHeading(new Pose2d(-35.96, 12.45, Math.toRadians(270.00)))
                        .lineToSplineHeading(new Pose2d(49.71, 12.80, Math.toRadians(0.00)))
                        .build();

                TrajectorySequence lastBackdropRight = robot.driveSubsystem.trajectorySequenceBuilder(initialBackdropRight.end())
                        .lineToConstantHeading(new Vector2d(49.71, 30.00))
                        .build();

                TrajectorySequence parkRight = robot.driveSubsystem.trajectorySequenceBuilder(lastBackdropRight.end())
                        .lineToConstantHeading(new Vector2d(40.14, 30.00))
                        .lineToConstantHeading(new Vector2d(39.44, 13.15))
                        .splineTo(new Vector2d(66.25, 12.45), Math.toRadians(0.00))
                        .build();

                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequencenotAsync(tapeRight)),
                                new WaitCommand(500),
                                new ParallelCommandGroup(
                                        new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequencenotAsync(initialBackdropRight)),
                                        new RestCommand(robot)
                                ),
                                new WaitCommand(500),
                                new OuttakeCommand(robot),
                                new WaitCommand(0), //edit this to wait before moving - track the time here if needed to find max
                                new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequencenotAsync(lastBackdropRight)),
                                new WaitCommand(1000),
                                new InstantCommand(() -> robot.claw.autoReleaseLeft()),
                                new WaitCommand(1000),
                                new ParallelCommandGroup(
                                        new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequencenotAsync(parkRight)),
                                        new GrabBothCommand(robot),
                                        new RestCommand(robot)
                                )

                        )
                );
                break;
            case MIDDLE:
                TrajectorySequence tapeMiddle = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-39.61, 68.34, Math.toRadians(270.00)))
                        .splineToConstantHeading(new Vector2d(-35.61, 32.82), Math.toRadians(270.00))
                        .build();

                TrajectorySequence initialBackdropMiddle = robot.driveSubsystem.trajectorySequenceBuilder(tapeMiddle.end())
                        .lineToConstantHeading(new Vector2d(-35.96, 46.06))
                        .lineToConstantHeading(new Vector2d(-49.71, 46.93))
                        .lineToSplineHeading(new Pose2d(-49.71, 12.80, Math.toRadians(270.00)))
                        .lineToSplineHeading(new Pose2d(49.71, 12.80, Math.toRadians(0.00)))
                        .build();

                TrajectorySequence lastBackdropMiddle = robot.driveSubsystem.trajectorySequenceBuilder(initialBackdropMiddle.end())
                        .lineToConstantHeading(new Vector2d(49.54, 35.43))
                        .build();

                TrajectorySequence parkMiddle = robot.driveSubsystem.trajectorySequenceBuilder(lastBackdropMiddle.end())
                        .lineToConstantHeading(new Vector2d(40.14, 35.26))
                        .lineToConstantHeading(new Vector2d(39.44, 13.15))
                        .splineTo(new Vector2d(66.25, 12.45), Math.toRadians(0.00))
                        .build();

                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequencenotAsync(tapeMiddle)),
                                new WaitCommand(500),
                                new ParallelCommandGroup(
                                        new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequencenotAsync(initialBackdropMiddle)),
                                        new RestCommand(robot)
                                ),
                                new WaitCommand(500),
                                new OuttakeCommand(robot),
                                new WaitCommand(0), //edit this to wait before moving - track the time here if needed to find max
                                new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequencenotAsync(lastBackdropMiddle)),
                                new WaitCommand(1000),
                                new InstantCommand(() -> robot.claw.autoReleaseLeft()),
                                new WaitCommand(1000),
                                new ParallelCommandGroup(
                                        new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequencenotAsync(parkMiddle)),
                                        new GrabBothCommand(robot),
                                        new RestCommand(robot)
                                )

                        )
                );
                break;
        }
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        robot.a.loop();
        robot.driveSubsystem.update();

        double time = System.currentTimeMillis();
        telemetry.addData("Time Elapsed: ", time_since_start);
        telemetry.addData("Current Loop Time: ", time - loop);
        telemetry.addData("Arm Position: ", robot.a.getCachePos());
        robot.currentUpdate(telemetry);
        loop = time;
        telemetry.update();
    }

    @Override
    public void stop() {
        colorMassDetectionProcessor.close();
        visionPortal.close();
        telemetry.addLine("Closed Camera.");
        telemetry.update();
        CommandScheduler.getInstance().reset();
    }
}