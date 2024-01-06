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
import com.qualcomm.robotcore.hardware.DcMotor;
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
import org.firstinspires.ftc.teamcode.util.RedRightProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

@Autonomous
@Config
public class HopeRedRight extends OpMode {
    private VisionPortal visionPortal;
    private RedRightProcessor colorMassDetectionProcessor;

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

        colorMassDetectionProcessor = new RedRightProcessor(
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
        telemetry.addData("Successful: ", "Ready for RedRight (Backdrop Side)");
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

        RedRightProcessor.PropPositions recordedPropPosition = colorMassDetectionProcessor.getRecordedPropPosition();
        robot.driveSubsystem.setPoseEstimate(new Pose2d(15.58, -63.82, Math.toRadians(90.00)));
        switch (recordedPropPosition) {
            case LEFT:
            case UNFOUND:
                TrajectorySequence backdropLeft = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(15.58, -63.82, Math.toRadians(90.00)))
                        .splineToSplineHeading(
                                new Pose2d(48.67, -29.86, Math.toRadians(0.00)), Math.toRadians(0.00),
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence tapeLeft = robot.driveSubsystem.trajectorySequenceBuilder(backdropLeft.end())
                        .lineToConstantHeading(new Vector2d(39.09, -30.38))
                        .lineToSplineHeading(
                                new Pose2d(12.45, -37.35, Math.toRadians(0.00)),
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence parkLeft = robot.driveSubsystem.trajectorySequenceBuilder(tapeLeft.end())
                        .splineTo(
                                new Vector2d(62.6, -58.94), Math.toRadians(0.00),
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new WaitCommand(200),
                                new ParallelCommandGroup(
                                        new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequencenotAsync(backdropLeft)),
                                        new OuttakeCommand(robot)
                                ),
                                new WaitCommand(2500),
                                new InstantCommand(() -> robot.claw.autoReleaseLeft()),
                                new WaitCommand(1000),
                                new InstantCommand(() -> robot.angle.stressor()),
                                new WaitCommand(1000),
                                new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequencenotAsync(tapeLeft)),
                                new GrabBothCommand(robot),
                                new WaitCommand(500),
                                        new TapeDropperCommand(robot),
                                new WaitCommand(1000),
                                new InstantCommand(() -> robot.claw.releaseRight()),
                                new WaitCommand(1000),
                                new RestCommand(robot),
                                new GrabBothCommand(robot),
                                new WaitCommand(1000),
                                new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequencenotAsync(parkLeft))
                        )
                );
                break;
            case RIGHT:
                TrajectorySequence backdropRight = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(15.58, -63.82, Math.toRadians(90.00)))
                        .splineToSplineHeading(
                                new Pose2d(49.54, -42.05, Math.toRadians(0.00)), Math.toRadians(0.00),
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence tapeRight = robot.driveSubsystem.trajectorySequenceBuilder(backdropRight.end())
                        .lineToConstantHeading(
                                new Vector2d(41.70, -42.05),
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToSplineHeading(
                                new Pose2d(34.04, -19.76, Math.toRadians(0.00)),
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence parkRight = robot.driveSubsystem.trajectorySequenceBuilder(tapeRight.end())
                        .splineTo(
                                new Vector2d(62.6, -58.94), Math.toRadians(0.00),
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new WaitCommand(200),
                                new ParallelCommandGroup(
                                        new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequencenotAsync(backdropRight)),
                                        new OuttakeCommand(robot)
                                ),
                                new WaitCommand(2500),
                                new InstantCommand(() -> robot.claw.autoReleaseLeft()),
                                new WaitCommand(1000),
                                new InstantCommand(() -> robot.angle.stressor()),
                                new WaitCommand(1000),
                                new ParallelCommandGroup(
                                        new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequencenotAsync(tapeRight)),
                                        new TapeDropperCommand(robot)
                                ),
                                new WaitCommand(1000),
                                new InstantCommand(() -> robot.claw.releaseRight()),
                                new WaitCommand(1000),
                                new RestCommand(robot),
                                new GrabBothCommand(robot),
                                new WaitCommand(1000),
                                new ParallelCommandGroup(
                                        new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequencenotAsync(parkRight)),
                                        new RestCommand(robot)
                                )
                        )
                );
                break;
            case MIDDLE:
                TrajectorySequence backdropMiddle = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(15.58, -63.82, Math.toRadians(90.00)))
                        .splineToSplineHeading(
                                new Pose2d(47.8, -36.83, Math.toRadians(0.00)), Math.toRadians(0.00),
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence tapeMiddle = robot.driveSubsystem.trajectorySequenceBuilder(backdropMiddle.end())
                        .lineToSplineHeading(
                                new Pose2d(27.42, -20.63, Math.toRadians(0.00)),
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence parkMiddle = robot.driveSubsystem.trajectorySequenceBuilder(tapeMiddle.end())
                        .splineTo(
                                new Vector2d(62.6, -58.94), Math.toRadians(0.00),
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new WaitCommand(200),
                                new ParallelCommandGroup(
                                        new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequencenotAsync(backdropMiddle)),
                                        new OuttakeCommand(robot)
                                ),
                                new WaitCommand(2500),
                                new InstantCommand(() -> robot.claw.autoReleaseLeft()),
                                new WaitCommand(1000),
                                new InstantCommand(() -> robot.angle.stressor()),
                                new WaitCommand(1000),
                                new ParallelCommandGroup(
                                        new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequencenotAsync(tapeMiddle)),
                                        new TapeDropperCommand(robot)
                                ),
                                new WaitCommand(1000),
                                new InstantCommand(() -> robot.claw.releaseRight()),
                                new WaitCommand(1000),
                                new RestCommand(robot),
                                new GrabBothCommand(robot),
                                new WaitCommand(1000),
                                new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequencenotAsync(parkMiddle))
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
        telemetry.addData("Linear 1 Position: ", robot.linear_1.getCurrentPosition());
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