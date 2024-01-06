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
import org.firstinspires.ftc.teamcode.common.commandbase.command.OuttakerCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.RestCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.TapeDropperCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.commandbase.command.OuttakeCommand;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.RedLeftProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

@Autonomous
@Config
public class HopeRedLeft extends OpMode {
    private VisionPortal visionPortal;
    private RedLeftProcessor colorMassDetectionProcessor;

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

        Scalar lower = new Scalar(0, 80, 80); // the lower hsv threshold
        Scalar upper = new Scalar(180, 250, 250); // the upper hsv threshold
        double minArea = 100;

        colorMassDetectionProcessor = new RedLeftProcessor(
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
        telemetry.addData("Successful: ", "Ready for RedLeft (Not Backdrop Side)");
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

        RedLeftProcessor.PropPositions recordedPropPosition = colorMassDetectionProcessor.getRecordedPropPosition();
        robot.driveSubsystem.setPoseEstimate(new Pose2d(-39.61, -63.99, Math.toRadians(90.00)));
        switch (recordedPropPosition) {
            case LEFT:
            case UNFOUND:
                TrajectorySequence tapeLeft = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-39.61, -63.99, Math.toRadians(90.00)))
                        .lineToConstantHeading(
                                new Vector2d(-52.85, -38.92),
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence initialBackdropLeft = robot.driveSubsystem.trajectorySequenceBuilder(tapeLeft.end())
                        .lineToConstantHeading(new Vector2d(-52.67, -51.98))
                        .lineToConstantHeading(new Vector2d(-33.87, -52.85))
                        .lineToConstantHeading(new Vector2d(-34.74, -12.10))
                        .lineToSplineHeading(
                                new Pose2d(-9.32, -11.58, Math.toRadians(359.08)),
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToSplineHeading(
                                new Pose2d(35.96, -13.15, Math.toRadians(360.00)),
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence lastBackdropLeft = robot.driveSubsystem.trajectorySequenceBuilder(initialBackdropLeft.end())
                        .lineToConstantHeading(
                                new Vector2d(35.09, -38.39),
                                SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToConstantHeading(
                                new Vector2d(50.41, -34.04),
                                SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence parkLeft = robot.driveSubsystem.trajectorySequenceBuilder(lastBackdropLeft.end())
                        .lineToConstantHeading(
                                new Vector2d(46.58, -34.22),
                                SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
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
                                new OuttakerCommand(robot),
                                new WaitCommand(3000), //edit this to wait before moving - track the time here if needed to find max
                                new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequencenotAsync(lastBackdropLeft)),
                                new WaitCommand(1000),
                                new InstantCommand(() -> robot.claw.autoReleaseLeft()),
                                new WaitCommand(1000),
                                new InstantCommand(() -> robot.angle.stressor()),
                                new WaitCommand(1000),
                                new ParallelCommandGroup(
                                        new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequencenotAsync(parkLeft)),
                                        new RestCommand(robot)
                                ),
                                new WaitCommand(1000)

                        )
                );
                break;
            case RIGHT:
                TrajectorySequence tapeRight = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-39.61, -63.99, Math.toRadians(90.00)))
                        .splineTo(
                                new Vector2d(-33.00, -35.78), Math.toRadians(22.00),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence initialBackdropRight = robot.driveSubsystem.trajectorySequenceBuilder(tapeRight.end())
                        .lineToConstantHeading(
                                new Vector2d(-43.97, -42.57),
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToSplineHeading(new Pose2d(-48.32, -7.92, Math.toRadians(0.00)))
                        .lineToSplineHeading(
                                new Pose2d(35.96, -6.88, Math.toRadians(360.00)),
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence lastBackdropRight = robot.driveSubsystem.trajectorySequenceBuilder(initialBackdropRight.end())
                        .lineToConstantHeading(
                                new Vector2d(35.09, -38.39),
                                SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .lineToConstantHeading(
                                new Vector2d(50.58, -43.1),
                                SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence parkRight = robot.driveSubsystem.trajectorySequenceBuilder(lastBackdropRight.end())
                        .lineToConstantHeading(
                                new Vector2d(44.84, -34.22),
                                SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
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
                                new OuttakerCommand(robot),
                                new WaitCommand(3000), //edit this to wait before moving - track the time here if needed to find max
                                new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequencenotAsync(lastBackdropRight)),
                                new WaitCommand(1000),
                                new InstantCommand(() -> robot.claw.autoReleaseLeft()),
                                new WaitCommand(1000),
                                new InstantCommand(() -> robot.angle.stressor()),
                                new WaitCommand(1000),
                                new ParallelCommandGroup(
                                        new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequencenotAsync(parkRight)),
                                        new RestCommand(robot)
                                ),
                                new WaitCommand(1000)

                        )
                );
                break;
            case MIDDLE:
                TrajectorySequence tapeMiddle = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-39.61, 68.34, Math.toRadians(270.00)))
                        .lineToConstantHeading(
                                new Vector2d(-44.66, 38.57),
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence initialBackdropMiddle = robot.driveSubsystem.trajectorySequenceBuilder(tapeMiddle.end())
                        .lineToConstantHeading(new Vector2d(-44.66, 50.41))
                        .lineToConstantHeading(new Vector2d(-53.72, 50.23))
                        .lineToConstantHeading(new Vector2d(-53.19, 16.28))
                        .lineToSplineHeading(
                                new Pose2d(-22.72, 15.58, Math.toRadians(0.92)),
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToSplineHeading(
                                new Pose2d(35.96, 13.15, Math.toRadians(0.00)),
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence lastBackdropMiddle = robot.driveSubsystem.trajectorySequenceBuilder(initialBackdropMiddle.end())
                        .lineToConstantHeading(
                                new Vector2d(34.74, 40.83),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToConstantHeading(
                                new Vector2d(49.7, 42.50),
                                SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence parkMiddle = robot.driveSubsystem.trajectorySequenceBuilder(lastBackdropMiddle.end())
                        .lineToConstantHeading(
                                new Vector2d(44.66, 40.83),
                                SampleMecanumDrive.getVelocityConstraint(12, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
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
                                new WaitCommand(3000), //edit this to wait before moving - track the time here if needed to find max
                                new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequencenotAsync(lastBackdropMiddle)),
                                new WaitCommand(1000),
                                new InstantCommand(() -> robot.claw.autoReleaseLeft()),
                                new WaitCommand(1000),
                                new InstantCommand(() -> robot.angle.stressor()),
                                new WaitCommand(1000),
                                new ParallelCommandGroup(
                                        new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequencenotAsync(parkMiddle)),
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