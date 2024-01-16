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
import org.firstinspires.ftc.teamcode.Utility.Vision.RedLeftProcessor;
import org.firstinspires.ftc.teamcode.Utility.Vision.RedRightProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

@Autonomous
@Config
public class RedLeft extends OpMode {
    private VisionPortal visionPortal;
    private RedLeftProcessor colorMassDetectionProcessor;

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

        Scalar lower = new Scalar(0, 80, 80);
        Scalar upper = new Scalar(180, 250, 250);
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

        RedLeftProcessor.PropPositions recordedPropPosition = colorMassDetectionProcessor.getRecordedPropPosition();
        robot.driveSubsystem.setPoseEstimate(new Pose2d(-39.96, -63.99, Math.toRadians(90.00)));

        switch (recordedPropPosition) {
            case LEFT:
            case UNFOUND:
                TrajectorySequence tapeLeft = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-39.96, -63.99, Math.toRadians(90.00)))
                        .splineToConstantHeading(new Vector2d(-50.41, -41.53), Math.toRadians(90.00))
                        .build();

                TrajectorySequence toWaitLeft = robot.driveSubsystem.trajectorySequenceBuilder(tapeLeft.end())
                        .lineToConstantHeading(new Vector2d(-50.58, -49.19))
                        .lineToConstantHeading(
                                new Vector2d(-37.52, -49.36),
                                SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToConstantHeading(new Vector2d(-36.3, -11.93))
                        .lineToSplineHeading(new Pose2d(8.79, -11.58, Math.toRadians(0.00)))
                        .lineToConstantHeading(new Vector2d(39.96, -11.75))
                        .build();

                TrajectorySequence toBackBoardLeft = robot.driveSubsystem.trajectorySequenceBuilder(toWaitLeft.end())
                        .lineToConstantHeading(
                                new Vector2d(49.54, -30.73),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence toParkLeft = robot.driveSubsystem.trajectorySequenceBuilder(toBackBoardLeft.end())
                        .lineToConstantHeading(new Vector2d(39.44, -31.26))
                        .splineTo(
                                new Vector2d(49.54, -15.24), Math.toRadians(0.00),
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new DriveCommand(robot.driveSubsystem, tapeLeft),
                                //new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequenceNotAsync(tapeLeft)),
                                new WaitCommand(350),
                                new DriveCommand(robot.driveSubsystem, toWaitLeft),
                                //new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequenceNotAsync(toWaitLeft)),
                                new WaitCommand(350), //this can be changed to maximize auto time!
                                new ParallelCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, toBackBoardLeft),
                                        //new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequenceNotAsync(toBackBoardLeft)),
                                        new OuttakeCommand(robot)
                                ),
                                new WaitCommand(350),
                                new InstantCommand(() -> robot.claw.autoReleaseLeft()),
                                new WaitCommand(1000),
                                new ParallelCommandGroup(
                                        new RestCommand(robot),
                                        new DriveCommand(robot.driveSubsystem, toParkLeft)
                                        //new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequenceNotAsync(toParkLeft))
                                )
                        )
                );
                break;
            case RIGHT:
                TrajectorySequence tapeRight = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-39.96, -63.99, Math.toRadians(90.00)))
                        .splineTo(
                                new Vector2d(-32.13, -34.39), Math.toRadians(20.00),
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence toWaitRight = robot.driveSubsystem.trajectorySequenceBuilder(tapeRight.end())
                        .lineToConstantHeading(new Vector2d(-46.93, -42.05))
                        .lineToSplineHeading(new Pose2d(-46.06, -11.41, Math.toRadians(0.00)))
                        .lineToConstantHeading(new Vector2d(39.96, -11.75))
                        .build();

                TrajectorySequence toBackBoardRight = robot.driveSubsystem.trajectorySequenceBuilder(toWaitRight.end())
                        .lineToConstantHeading(
                                new Vector2d(49.54, -46.06),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence toParkRight = robot.driveSubsystem.trajectorySequenceBuilder(toBackBoardRight.end())
                        .lineToConstantHeading(new Vector2d(39.96, -46.40))
                        .splineTo(
                                new Vector2d(49.54, -15.24), Math.toRadians(0.00),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new DriveCommand(robot.driveSubsystem, tapeRight),
                                //new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequenceNotAsync(tapeRight)),
                                new WaitCommand(350),
                                new DriveCommand(robot.driveSubsystem, toWaitRight),
                                //new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequenceNotAsync(toWaitRight)),
                                new WaitCommand(350), //this can be changed to maximize auto time!
                                new ParallelCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, toBackBoardRight),
                                        //new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequenceNotAsync(toBackBoardRight)),
                                        new OuttakeCommand(robot)
                                ),
                                new WaitCommand(350),
                                new InstantCommand(() -> robot.claw.autoReleaseLeft()),
                                new WaitCommand(1000),
                                new ParallelCommandGroup(
                                        new RestCommand(robot),
                                        new DriveCommand(robot.driveSubsystem, toParkRight)
                                        //new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequenceNotAsync(toParkRight))
                                )
                        )
                );

                break;
            case MIDDLE:
                TrajectorySequence tapeMiddle = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-39.96, -63.99, Math.toRadians(90.00)))
                        .splineToConstantHeading(
                                new Vector2d(-34.56, -34.39), Math.toRadians(90.00),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence toWaitMiddle = robot.driveSubsystem.trajectorySequenceBuilder(tapeMiddle.end())
                        .lineToSplineHeading(
                                new Pose2d(-37.00, -44.66, Math.toRadians(90.00)),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToSplineHeading(new Pose2d(-55.28, -44.84, Math.toRadians(90.00)))
                        .lineToSplineHeading(new Pose2d(-54.59, -13.32, Math.toRadians(89.70)))
                        .lineToSplineHeading(new Pose2d(40.48, -19.41, Math.toRadians(0.00)))
                        .build();

                TrajectorySequence toBackBoardMiddle = robot.driveSubsystem.trajectorySequenceBuilder(toWaitMiddle.end())
                        .lineToSplineHeading(new Pose2d(40.31, -43.79, Math.toRadians(0.00)))
                        .lineToConstantHeading(
                                new Vector2d(47.8, -43.62),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence toParkMiddle = robot.driveSubsystem.trajectorySequenceBuilder(toBackBoardMiddle.end())
                        .lineToConstantHeading(new Vector2d(42.4, -43.79))
                        .lineToConstantHeading(new Vector2d(42.22, -18.37))
                        .lineToConstantHeading(
                                new Vector2d(56.85, -18.02),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new DriveCommand(robot.driveSubsystem, tapeMiddle),
                                //new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequenceNotAsync(tapeMiddle)),
                                new WaitCommand(350),
                                new DriveCommand(robot.driveSubsystem, toWaitMiddle),
                                //new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequenceNotAsync(toWaitMiddle)),
                                new WaitCommand(1000), //this can be changed to maximize auto time!
                                new DriveCommand(robot.driveSubsystem, toBackBoardMiddle),
                                new WaitCommand(350),
                                new OuttakeCommand(robot),
                                new WaitCommand(1500),
                                new InstantCommand(() -> robot.claw.autoReleaseLeft()),
                                new WaitCommand(1000),
                                new ParallelCommandGroup(
                                        new TapeDropCommand(robot),
                                        new DriveCommand(robot.driveSubsystem, toParkMiddle)
                                        //new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequenceNotAsync(toParkMiddle))
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