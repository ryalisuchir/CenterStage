package org.firstinspires.ftc.teamcode.drive.opmode.APerfectWorld.Backups;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.commandbase.command.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.ColorPropDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

@Autonomous
@Config
public class APWRedLeftBackup extends OpMode {
    private VisionPortal visionPortal;
    private ColorPropDetectionProcessor colorMassDetectionProcessor;

    private Robot robot;
    private double loop;
    private ElapsedTime time_since_start;

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
        Scalar lower = new Scalar(0, 80, 80); // the lower hsv threshold
        Scalar upper = new Scalar(180, 250, 250); // the upper hsv threshold
        double minArea = 100; //min area for prop detection

        colorMassDetectionProcessor = new ColorPropDetectionProcessor(
                lower,
                upper,
                () -> minArea,
                () -> 213, //left third
                () -> 426 //right third
        );
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .addProcessor(colorMassDetectionProcessor)
                .build();

    }

    @Override
    public void init_loop() {
        telemetry.addData("Successful: ", "Ready for RedLeft (Backdrop Side)");
        telemetry.addData("Ready to Run: ", "1 pixel autonomous (26 points). All subsystems initialized.");
        telemetry.addData("Currently Recorded Position", colorMassDetectionProcessor.getRecordedPropPosition());
        telemetry.addData("Camera State", visionPortal.getCameraState());
        telemetry.addData("Currently Detected Mass Center", "x: " + colorMassDetectionProcessor.getLargestContourX() + ", y: " + colorMassDetectionProcessor.getLargestContourY());
        telemetry.addData("Currently Detected Mass Area", colorMassDetectionProcessor.getLargestContourArea());
        CommandScheduler.getInstance().run();
        robot.a.loop();
    }

    @Override
    public void start() {
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.stopLiveView();
            visionPortal.stopStreaming();
        }

        ColorPropDetectionProcessor.PropPositions recordedPropPosition = colorMassDetectionProcessor.getRecordedPropPosition();
        robot.driveSubsystem.setPoseEstimate(new Pose2d(-32.13, -63.82, Math.toRadians(90.00)));
        switch (recordedPropPosition) {
            case LEFT:
            case UNFOUND:
                TrajectorySequence tapeLeft = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-32.13, -63.82, Math.toRadians(90.00)))
                        .splineToConstantHeading(new Vector2d(-51.63, -39.96), Math.toRadians(90.00))
                        .build();

                TrajectorySequence parkLeft = robot.driveSubsystem.trajectorySequenceBuilder(tapeLeft.end())
                        .lineToConstantHeading(new Vector2d(-51.28, -52.67))
                        .lineToConstantHeading(new Vector2d(-36.65, -53.02))
                        .lineToSplineHeading(new Pose2d(-36.13, -8.27, Math.toRadians(0.00)))
                        .splineTo(new Vector2d(63.64, -13.67), Math.toRadians(0.00))
                        .build();

                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequencenotAsync(tapeLeft)),
                                new WaitCommand(2000),
                                new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequencenotAsync(parkLeft)),
                                new WaitCommand(1000),
                                new IntakeCommand(robot)
                        )
                );
                break;
            case MIDDLE:
                TrajectorySequence tapeMiddle = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(12.45, 66.78, Math.toRadians(270.00)))
                        .splineToConstantHeading(new Vector2d(-36.48, -30.91), Math.toRadians(90.00))
                        .build();

                TrajectorySequence parkMiddle = robot.driveSubsystem.trajectorySequenceBuilder(tapeMiddle.end())
                        .lineToConstantHeading(new Vector2d(-36.65, -42.05))
                        .lineToConstantHeading(new Vector2d(-52.67, -42.05))
                        .lineToSplineHeading(new Pose2d(-52.15, -4.96, Math.toRadians(0.00)))
                        .splineTo(new Vector2d(63.64, -13.67), Math.toRadians(0.00))
                        .build();

                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequencenotAsync(tapeMiddle)),
                                new WaitCommand(2000),
                                new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequencenotAsync(parkMiddle)),
                                new WaitCommand(1000),
                                new IntakeCommand(robot)
                        )
                );

                break;
            case RIGHT:
                TrajectorySequence tapeRight = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(12.45, 66.78, Math.toRadians(270.00)))
                        .splineToConstantHeading(new Vector2d(-38.05, -48.67), Math.toRadians(90.00))
                        .splineTo(new Vector2d(-31.60, -37.87), Math.toRadians(56.31))
                        .build();

                TrajectorySequence parkRight = robot.driveSubsystem.trajectorySequenceBuilder(tapeRight.end())
                        .lineToConstantHeading(new Vector2d(-40.48, -49.89))
                        .lineToSplineHeading(new Pose2d(-40.31, -7.40, Math.toRadians(0.00)))
                        .splineTo(new Vector2d(63.82, -12.97), Math.toRadians(0.00))
                        .build();

                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequencenotAsync(tapeRight)),
                                new WaitCommand(2000),
                                new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequencenotAsync(parkRight)),
                                new WaitCommand(1000),
                                new IntakeCommand(robot)
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
        telemetry.addData("Loop: ", time - loop);
        telemetry.addData("Arm Position: ", robot.a.getCachePos());
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