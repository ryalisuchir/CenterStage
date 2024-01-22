package org.firstinspires.ftc.teamcode.Drive.OpModes.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.Drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.TrajectorySequences.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.DriveCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.OuttakeCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.RestCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.SpecialOuttakeCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.TapeDropCommand;
import org.firstinspires.ftc.teamcode.Utility.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.Vision.BlueLeftProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

//USE IF ALL ELSE FAILS
@Autonomous
@Config
@Disabled

public class BlueLeft extends OpMode {
    private VisionPortal visionPortal;
    private BlueLeftProcessor colorMassDetectionProcessor;

    private RobotHardware robot;
    private ElapsedTime time_since_start;
    private double loop;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        robot = new RobotHardware(hardwareMap);

        CommandScheduler.getInstance().registerSubsystem(robot.armSystem);
        CommandScheduler.getInstance().registerSubsystem(robot.claw);
        CommandScheduler.getInstance().registerSubsystem(robot.angleOfArm);
        CommandScheduler.getInstance().registerSubsystem(robot.driveSubsystem);

        telemetry.addData("Not Ready: ", "Not able to proceed to camera detection... Restart robot now.");
        telemetry.update();

        robot.claw.grabBoth();

        Scalar lower = new Scalar(107.7, 143.1, 38.3);
        Scalar upper = new Scalar(119, 229.5, 133.3);
        double minArea = 100;

        colorMassDetectionProcessor = new BlueLeftProcessor(
                lower,
                upper,
                () -> minArea,
                () -> 213, //left third of frame
                () -> 426, //right third of frame
                400
        );
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .addProcessor(colorMassDetectionProcessor)
                .build();
    }

    @Override
    public void init_loop() {
        telemetry.addData("Successful: ", "Ready for BlueLeft (Backdrop Side)");
        telemetry.addData("Ready to Run: ", "2 pixel autonomous. All subsystems initialized.");
        telemetry.addData("Currently Recorded Position", colorMassDetectionProcessor.getRecordedPropPosition());
        telemetry.addData("Camera State", visionPortal.getCameraState());
        telemetry.addData("Currently Detected Mass Center", "x: " + colorMassDetectionProcessor.getLargestContourX() + ", y: " + colorMassDetectionProcessor.getLargestContourY());
        telemetry.addData("Currently Detected Mass Area", colorMassDetectionProcessor.getLargestContourArea());
        CommandScheduler.getInstance().run();
        robot.armSystem.loop();
    }

    @Override
    public void start() {
        time_since_start = new ElapsedTime();
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.stopLiveView();
            visionPortal.stopStreaming();
        }

        BlueLeftProcessor.PropPositions recordedPropPosition = colorMassDetectionProcessor.getRecordedPropPosition();
        robot.driveSubsystem.setPoseEstimate(new Pose2d(15.76, 63.99, Math.toRadians(-90.00)));
        switch (recordedPropPosition) {
            case LEFT:
            case UNFOUND:
                TrajectorySequence backdropLeft = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(15.76, 63.99, Math.toRadians(-90.00)))
                        .splineTo(
                                new Vector2d(48.5, 38), Math.toRadians(0.00),
                                SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence tapeLeft = robot.driveSubsystem.trajectorySequenceBuilder(backdropLeft.end())
                        .lineToSplineHeading(
                                new Pose2d(32.5, 38.39, Math.toRadians(0.00)),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence parkLeft = robot.driveSubsystem.trajectorySequenceBuilder(tapeLeft.end())
                        .lineToConstantHeading(new Vector2d(38.22, 38.05))
                        .lineToConstantHeading(new Vector2d(36.65, 58.59))
                        .lineToConstantHeading(new Vector2d(58.94, 57.90))
                        .build();

                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, backdropLeft),
//                               new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequenceNotAsync(backdropLeft)),
                                        new SpecialOuttakeCommand(robot)
                                ),
                                new WaitCommand(350),
                                new InstantCommand(() -> robot.claw.autoReleaseLeft()),
                                new WaitCommand(500),
                                new ParallelCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, tapeLeft),
                                        //new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequenceNotAsync(tapeLeft)),
                                        new TapeDropCommand(robot)
                                ),
                                new WaitCommand(350),
                                new InstantCommand(() -> robot.claw.releaseRight()),
                                new WaitCommand(1000),
                                new RestCommand(robot),
                                new WaitCommand(350),
                                new DriveCommand(robot.driveSubsystem, parkLeft)
                                //new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequenceNotAsync(parkLeft))

                        )
                );
                break;
            case RIGHT:
                TrajectorySequence backdropRight = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(15.76, 63.99, Math.toRadians(-90.00)))
                        .splineTo(
                                new Vector2d(48.6, 26.4), Math.toRadians(0.00),
                                SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence tapeRight = robot.driveSubsystem.trajectorySequenceBuilder(backdropRight.end())
                        .lineToSplineHeading(
                                new Pose2d(11.41, 39.5, Math.toRadians(0.00)),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence parkRight = robot.driveSubsystem.trajectorySequenceBuilder(tapeRight.end())
                        .lineToConstantHeading(new Vector2d(19.07, 38.05))
//                        .lineToConstantHeading(new Vector2d(34.39, 58.42))
                        .splineToConstantHeading(
                                new Vector2d(58.07, 63.12), Math.toRadians(0.00),
                                SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, backdropRight),
                                        //new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequenceNotAsync(backdropRight)),
                                        new SpecialOuttakeCommand(robot)
                                ),
                                new WaitCommand(350),
                                new InstantCommand(() -> robot.claw.autoReleaseLeft()),
                                new WaitCommand(500),
                                new ParallelCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, tapeRight),
                                        //new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequenceNotAsync(tapeRight)),
                                        new TapeDropCommand(robot)
                                ),
                                new WaitCommand(350),
                                new InstantCommand(() -> robot.claw.releaseRight()),
                                new WaitCommand(1000),
                                new RestCommand(robot),
                                new WaitCommand(350),
                                new DriveCommand(robot.driveSubsystem, parkRight)
                                //new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequenceNotAsync(parkRight))

                        )
                );
                break;
            case MIDDLE:
                TrajectorySequence backdropMiddle = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(15.76, 63.99, Math.toRadians(-90.00)))
                        .splineTo(
                                new Vector2d(49.2, 32.5), Math.toRadians(0.00),
                                SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence tapeMiddle = robot.driveSubsystem.trajectorySequenceBuilder(backdropMiddle.end())
                        .lineToSplineHeading(
                                new Pose2d(26.5, 27.77, Math.toRadians(0.00)),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence parkMiddle = robot.driveSubsystem.trajectorySequenceBuilder(tapeMiddle.end())
                        .lineToConstantHeading(
                                new Vector2d(34.91, 27.77)
                        )
                        .lineToConstantHeading(
                                new Vector2d(34.74, 60),
                                SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToConstantHeading(
                                new Vector2d(56.33, 60),
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        //new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequenceNotAsync(backdropMiddle)),
                                        new DriveCommand(robot.driveSubsystem, backdropMiddle),
                                        new SpecialOuttakeCommand(robot)
                                ),
                                new WaitCommand(350),
                                new InstantCommand(() -> robot.claw.autoReleaseLeft()),
                                new WaitCommand(500),
                                new ParallelCommandGroup(
                                        //new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequenceNotAsync(tapeMiddle)),
                                        new DriveCommand(robot.driveSubsystem, tapeMiddle),
                                        new TapeDropCommand(robot)
                                ),
                                new WaitCommand(350),
                                new InstantCommand(() -> robot.claw.releaseRight()),
                                new WaitCommand(1000),
                                new RestCommand(robot),
                                new WaitCommand(350),
                                new DriveCommand(robot.driveSubsystem, parkMiddle)
                                //new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequenceNotAsync(parkMiddle))

                        )
                );

                break;
        }
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        robot.armSystem.loop();
        robot.slidesSubsystem.loop();
        robot.driveSubsystem.update();

        double time = System.currentTimeMillis();
        telemetry.addData("Time Elapsed: ", time_since_start);
        telemetry.addData("Current Loop Time: ", time - loop);

        robot.currentUpdate(telemetry);
        robot.pidArmUpdateTelemetry(telemetry);

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