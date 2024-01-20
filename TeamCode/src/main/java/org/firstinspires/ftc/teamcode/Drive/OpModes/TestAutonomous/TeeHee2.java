package org.firstinspires.ftc.teamcode.Drive.OpModes.TestAutonomous;

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
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.StackCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.TapeDropCommand;
import org.firstinspires.ftc.teamcode.Utility.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.Vision.BlueLeftProcessor;
import org.firstinspires.ftc.teamcode.Utility.Vision.RedRightProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

//USE IF ALL ELSE FAILS
@Autonomous
@Config

public class TeeHee2 extends OpMode {
    private VisionPortal visionPortal;
    private RedRightProcessor colorMassDetectionProcessor;

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
    public void start() {
        time_since_start = new ElapsedTime();
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.stopLiveView();
            visionPortal.stopStreaming();
        }

        RedRightProcessor.PropPositions recordedPropPosition = colorMassDetectionProcessor.getRecordedPropPosition();
        robot.driveSubsystem.setPoseEstimate(new Pose2d(16.11, -64.34, Math.toRadians(90.00)));

        switch (recordedPropPosition) {
            case LEFT:
            case UNFOUND:
            case RIGHT:
            case MIDDLE:
                TrajectorySequence movement1Right = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(16.11, -64.34, Math.toRadians(90.00)))
                        .splineToConstantHeading(new Vector2d(19.24, -31.78), Math.toRadians(90.00))
                        .splineToConstantHeading(new Vector2d(19.07, -53.54), Math.toRadians(90.00))
                        .splineTo(new Vector2d(50.76, -39.09), Math.toRadians(0.00))
                        .build();

                TrajectorySequence movement2Right = robot.driveSubsystem.trajectorySequenceBuilder(movement1Right.end())
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(32.13, -11.75), Math.toRadians(180.00))
                        .splineToConstantHeading(
                                new Vector2d(-35.61, -11.58), Math.toRadians(180.00),
                                SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .splineToConstantHeading(new Vector2d(-58.24, -11.75), Math.toRadians(180.00))
                        .build();

                TrajectorySequence movement3Right = robot.driveSubsystem.trajectorySequenceBuilder(movement2Right.end())
                        .setReversed(false)
                        .splineToConstantHeading(new Vector2d(32.13, -12.10), Math.toRadians(0.00))
                        .splineToConstantHeading(new Vector2d(51.28, -39.44), Math.toRadians(0.00))
                        .build();

                TrajectorySequence movement4Right= robot.driveSubsystem.trajectorySequenceBuilder(movement3Right.end())
                        .setReversed(true)
                        .splineTo(new Vector2d(41.35, -39.61), Math.toRadians(180.00))
                        .splineToConstantHeading(new Vector2d(57.55, -62.07), Math.toRadians(0.00))
                        .build();

                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new DriveCommand(robot.driveSubsystem, movement1Right),
                                new OuttakeCommand(robot),
                                new WaitCommand(500),
                                new InstantCommand(() -> robot.claw.releaseLeft()),
                                new WaitCommand(350),
                                new ParallelCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement2Right),
                                        new StackCommand(robot)
                                ),
                                new WaitCommand(350),
                                new InstantCommand(() -> robot.claw.grabBoth()),
                                new WaitCommand(350),
                                new ParallelCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement3Right),
                                        new RestCommand(robot)
                                ),
                                new WaitCommand(350),
                                new OuttakeCommand(robot),
                                new WaitCommand(350),
                                new InstantCommand(() -> robot.claw.releaseLeft()),
                                new WaitCommand(350),
                                new ParallelCommandGroup(
                                        new TapeDropCommand(robot),
                                        new DriveCommand(robot.driveSubsystem, movement4Right)
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
        telemetry.update();
        CommandScheduler.getInstance().reset();
    }
}