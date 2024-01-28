package org.firstinspires.ftc.teamcode.Drive.OpModes.Autonomous;

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
import org.firstinspires.ftc.teamcode.Utility.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.Vision.BlueLeftProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

@Autonomous
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
        telemetry.addData("Currently Recorded Position: ", colorMassDetectionProcessor.getRecordedPropPosition());
        telemetry.addData("Camera State: ", visionPortal.getCameraState());
        telemetry.addData("Currently Detected Mass Area: ", colorMassDetectionProcessor.getLargestContourArea());
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
        robot.driveSubsystem.setPoseEstimate(new Pose2d(16.14, 63.32, Math.toRadians(-90.00)));
        switch (recordedPropPosition) {
            case LEFT:
                TrajectorySequence movement1Left = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(16.14, 63.32, Math.toRadians(-90.00)))
                        .splineToConstantHeading(
                                new Vector2d(27.8, 43.31), Math.toRadians(270.00),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToConstantHeading(
                                new Vector2d(27.52, 56.52),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence movement2Left = robot.driveSubsystem.trajectorySequenceBuilder(movement1Left.end())
                        .splineToSplineHeading(
                                new Pose2d(52.3, 41, Math.toRadians(0.00)), Math.toRadians(0.00),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence movement3Left = robot.driveSubsystem.trajectorySequenceBuilder(movement2Left.end())
                        .lineToConstantHeading(
                                new Vector2d(38.41, 42),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence movement4Left = robot.driveSubsystem.trajectorySequenceBuilder(movement3Left.end())
                        .lineToConstantHeading(
                                new Vector2d(39.06, 60),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToConstantHeading(
                                new Vector2d(58, 60),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new DriveCommand(robot.driveSubsystem, movement1Left),
                                new ParallelCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement2Left),
                                        new OuttakeCommand(robot)
                                ),
                                new WaitCommand(750),
                                new InstantCommand(() -> robot.claw.releaseLeft()),
                                new WaitCommand(750),
                                new DriveCommand(robot.driveSubsystem, movement3Left),
                                new ParallelCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement4Left),
                                        new RestCommand(robot)
                                )

                        )
                );
                break;
            case RIGHT:
            case UNFOUND:
                TrajectorySequence movement1Right = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(16.14, 63.32, Math.toRadians(-90.00)))
                        .splineTo(
                                new Vector2d(9.8, 35.78), Math.toRadians(220.00),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToSplineHeading(
                                new Pose2d(19.31, 46.84, Math.toRadians(270.00)),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence movement2Right = robot.driveSubsystem.trajectorySequenceBuilder(movement1Right.end())
                        .splineToSplineHeading(
                                new Pose2d(51.8, 28.4, Math.toRadians(0.00)), Math.toRadians(0.00),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence movement3Right = robot.driveSubsystem.trajectorySequenceBuilder(movement2Right.end())
                        .lineToConstantHeading(
                                new Vector2d(38.41, 28.4),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence movement4Right = robot.driveSubsystem.trajectorySequenceBuilder(movement3Right.end())
                        .lineToConstantHeading(
                                new Vector2d(39.06, 60),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToConstantHeading(
                                new Vector2d(58, 60),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new DriveCommand(robot.driveSubsystem, movement1Right),
                                new ParallelCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement2Right),
                                        new OuttakeCommand(robot)
                                ),
                                new WaitCommand(750),
                                new InstantCommand(() -> robot.claw.releaseLeft()),
                                new WaitCommand(750),
                                new DriveCommand(robot.driveSubsystem, movement3Right),
                                new ParallelCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement4Right),
                                        new RestCommand(robot)
                                )

                        )
                );
                break;
            case MIDDLE:
                TrajectorySequence movement1Middle = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(16.14, 63.32, Math.toRadians(-90.00)))
                        .splineToConstantHeading(
                                new Vector2d(20.25, 33.88), Math.toRadians(270.00),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToConstantHeading(
                                new Vector2d(20.25, 50.07),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence movement2Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement1Middle.end())
                        .splineToSplineHeading(
                                new Pose2d(52.6, 33.4, Math.toRadians(0.00)), Math.toRadians(0.00),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence movement3Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement2Middle.end())
                        .lineToConstantHeading(
                                new Vector2d(38.41, 33.4),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence movement4Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement3Middle.end())
                        .lineToConstantHeading(
                                new Vector2d(39.06, 60),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToConstantHeading(
                                new Vector2d(58, 60),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new DriveCommand(robot.driveSubsystem, movement1Middle),
                                new ParallelCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement2Middle),
                                        new OuttakeCommand(robot)
                                ),
                                new WaitCommand(750),
                                new InstantCommand(() -> robot.claw.releaseLeft()),
                                new WaitCommand(750),
                                new DriveCommand(robot.driveSubsystem, movement3Middle),
                                new ParallelCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement4Middle),
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