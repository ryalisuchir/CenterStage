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
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.DropperSometimes;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.HigherOuttakeCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.OuttakeCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.RestCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.SpecialOuttakeCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.TapeDropCommand;
import org.firstinspires.ftc.teamcode.Utility.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.Vision.BlueLeftProcessor;
import org.firstinspires.ftc.teamcode.Utility.Vision.BlueRightProcessor;
import org.firstinspires.ftc.teamcode.Utility.Vision.RedLeftProcessor;
import org.firstinspires.ftc.teamcode.Utility.Vision.RedRightProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

//HAVE NOT TESTED. EDITING MIDDLE.
@Autonomous
@Config
@Disabled

public class BlueRight extends OpMode {
    private VisionPortal visionPortal;
    private BlueRightProcessor colorMassDetectionProcessor;

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

        colorMassDetectionProcessor = new BlueRightProcessor(
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
        telemetry.addData("Successful: ", "Ready for BlueRight (Backdrop Side)");
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

        BlueRightProcessor.PropPositions recordedPropPosition = colorMassDetectionProcessor.getRecordedPropPosition();
        robot.driveSubsystem.setPoseEstimate(new Pose2d(-41.01, 65.38, Math.toRadians(270.00)));

        switch (recordedPropPosition) {
            case LEFT:
                TrajectorySequence tapeLeft = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-41.01, 65.38, Math.toRadians(-90)))
                        .splineTo(
                                new Vector2d(-34.22, 42.05), Math.toRadians(-60.00),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence toWaitLeft = robot.driveSubsystem.trajectorySequenceBuilder(tapeLeft.end())
                        .setReversed(true)
                        .lineToSplineHeading(
                                new Pose2d(-41.70, 49.19, Math.toRadians(-90.00)),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToSplineHeading(
                                new Pose2d(-57.2, 49.71, Math.toRadians(-90.00)),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .setReversed(false)
                        .lineToSplineHeading(
                                new Pose2d(-56.68, 16.37, Math.toRadians(-90.00)),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToSplineHeading(
                                new Pose2d(-8.27, 12.28, Math.toRadians(0.00)),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToLinearHeading(
                                new Pose2d(35.26, 11.23, Math.toRadians(0.00)),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence toBackBoardLeft = robot.driveSubsystem.trajectorySequenceBuilder(toWaitLeft.end())
                        .strafeLeft(26.5)
                        .forward(12.5,
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence toParkLeft = robot.driveSubsystem.trajectorySequenceBuilder(toBackBoardLeft.end())
                        .forward(-5)
                        .build();

                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new DriveCommand(robot.driveSubsystem, tapeLeft),
                                new WaitCommand(350),
                                new DriveCommand(robot.driveSubsystem, toWaitLeft),
                                new WaitCommand(5500), //this can be changed to maximize auto time!
                                new DriveCommand(robot.driveSubsystem, toBackBoardLeft),
                                new WaitCommand(350),
                                new HigherOuttakeCommand(robot),
                                new WaitCommand(1500),
                                new InstantCommand(() -> robot.claw.autoReleaseLeft()),
                                new WaitCommand(1000),
                                new ParallelCommandGroup(
                                        new DropperSometimes(robot),
                                        new DriveCommand(robot.driveSubsystem, toParkLeft)
                                        //new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequenceNotAsync(toParkMiddle))
                                )
                        )
                );
                break;
            case UNFOUND:
            case RIGHT:
                TrajectorySequence tapeRight = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-41.01, 65.38, Math.toRadians(-90)))
                        .splineToConstantHeading(
                                new Vector2d(-52.5, 41.53), Math.toRadians(-90.00),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence toWaitRight = robot.driveSubsystem.trajectorySequenceBuilder(tapeRight.end())
                        .setReversed(true)
                        .lineToSplineHeading(new Pose2d(-52.5, 48.49, Math.toRadians(-90.00)))
                        .lineToConstantHeading(
                                new Vector2d(-36.83, 48.32),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .setReversed(false)
                        .splineToConstantHeading(
                                new Vector2d(-36.3, 13.15), (Math.toRadians(270.00)),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToSplineHeading(
                                new Pose2d(-2.00, 12.80, Math.toRadians(0.00)),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToLinearHeading(
                                new Pose2d(34.56, 12.80, Math.toRadians(0.00)),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence toBackBoardRight = robot.driveSubsystem.trajectorySequenceBuilder(toWaitRight.end())
                        .strafeLeft(15.5)
                        .forward(13,
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence toParkRight = robot.driveSubsystem.trajectorySequenceBuilder(toBackBoardRight.end())
                        .forward(-5)
                        .build();

                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new DriveCommand(robot.driveSubsystem, tapeRight),
                                new WaitCommand(350),
                                new DriveCommand(robot.driveSubsystem, toWaitRight),
                                new WaitCommand(9000), //this can be changed to maximize auto time!
                                new DriveCommand(robot.driveSubsystem, toBackBoardRight),
                                new WaitCommand(350),
                                new HigherOuttakeCommand(robot),
                                new WaitCommand(1500),
                                new InstantCommand(() -> robot.claw.autoReleaseLeft()),
                                new WaitCommand(1000),
                                new ParallelCommandGroup(
                                        new DropperSometimes(robot),
                                        new DriveCommand(robot.driveSubsystem, toParkRight)
                                        //new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequenceNotAsync(toParkMiddle))
                                )
                        )
                );
                break;
            case MIDDLE:
                TrajectorySequence tapeMiddle = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-41.01, 65.38, Math.toRadians(-90)))
                        .splineToConstantHeading(
                                new Vector2d(-45.36, 36.83), Math.toRadians(-90.00),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence toWaitMiddle = robot.driveSubsystem.trajectorySequenceBuilder(tapeMiddle.end())
                        .setReversed(true)
                        .lineToSplineHeading(
                                new Pose2d(-45.36, 51.11, Math.toRadians(-90.00)),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToSplineHeading(
                                new Pose2d(-57.03, 50.93, Math.toRadians(-90.00)),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .setReversed(false)
                        .lineToSplineHeading(
                                new Pose2d(-56.68, 12.62, Math.toRadians(-90.00)),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToSplineHeading(
                                new Pose2d(-9.49, 11.41, Math.toRadians(0.00)),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToLinearHeading(
                                new Pose2d(35.26, 11.23, Math.toRadians(0.00)),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence toBackBoardMiddle = robot.driveSubsystem.trajectorySequenceBuilder(toWaitMiddle.end())
                        .strafeLeft(19.5)
                        .forward(12.3,
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence toParkMiddle = robot.driveSubsystem.trajectorySequenceBuilder(toBackBoardMiddle.end())
                        .forward(-5)
                        .build();

                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new DriveCommand(robot.driveSubsystem, tapeMiddle),
                                new WaitCommand(350),
                                new DriveCommand(robot.driveSubsystem, toWaitMiddle),
                                new WaitCommand(6000), //this can be changed to maximize auto time!
                                new DriveCommand(robot.driveSubsystem, toBackBoardMiddle),
                                new WaitCommand(350),
                                new HigherOuttakeCommand(robot),
                                new WaitCommand(1500),
                                new InstantCommand(() -> robot.claw.autoReleaseLeft()),
                                new WaitCommand(1000),
                                new ParallelCommandGroup(
                                        new DropperSometimes(robot),
                                        new DriveCommand(robot.driveSubsystem, toParkMiddle)
                                )
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