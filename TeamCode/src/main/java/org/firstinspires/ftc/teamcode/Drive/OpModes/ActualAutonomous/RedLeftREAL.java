package org.firstinspires.ftc.teamcode.Drive.OpModes.ActualAutonomous;

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
public class RedLeftREAL extends OpMode {
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
        //telemetry.addData("Successful: ", "Ready for BlueLeft (Backdrop Side)");
        //telemetry.addData("Ready to Run: ", "2 pixel autonomous. All subsystems initialized.");
        telemetry.addData("Currently Recorded Position", colorMassDetectionProcessor.getRecordedPropPosition());
        //telemetry.addData("Camera State", visionPortal.getCameraState());
        //telemetry.addData("Currently Detected Mass Center", "x: " + colorMassDetectionProcessor.getLargestContourX() + ", y: " + colorMassDetectionProcessor.getLargestContourY());
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
        robot.driveSubsystem.setPoseEstimate(new Pose2d(-43.10, -64.51, Math.toRadians(90.00)));
        switch (recordedPropPosition) {
            case LEFT:
            case UNFOUND:
                TrajectorySequence movement1Left = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-43.10, -64.51, Math.toRadians(90.00)))
                        .splineToConstantHeading(new Vector2d(-49.54, -40.83), Math.toRadians(90.00))
                        .splineTo(new Vector2d(-49.54, -54.59), Math.toRadians(90.00))
                        .splineToConstantHeading(new Vector2d(-35.09, -45.53), Math.toRadians(90.00))
                        .splineToConstantHeading(new Vector2d(-34.74, -18.89), Math.toRadians(90.00))
                        .splineTo(new Vector2d(-4.79, -11.58), Math.toRadians(0.00))
                        .splineTo(new Vector2d(36.83, -11.58), Math.toRadians(0.00))
                        .build();

                TrajectorySequence movement2Left = robot.driveSubsystem.trajectorySequenceBuilder(movement1Left.end())
                        .splineToConstantHeading(new Vector2d(50.23, -33.69), Math.toRadians(0.00))
                        .build();

                TrajectorySequence movement3Left = robot.driveSubsystem.trajectorySequenceBuilder(movement2Left.end())
                        .splineTo(new Vector2d(40.66, -33.52), Math.toRadians(0.00))
                        .splineToConstantHeading(new Vector2d(59.11, -61.90), Math.toRadians(0.00))
                                .build();

                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new DriveCommand(robot.driveSubsystem, movement1Left),
                                new WaitCommand(350), //maximize
                                new ParallelCommandGroup(
                                        new OuttakeCommand(robot),
                                        new DriveCommand(robot.driveSubsystem, movement2Left)
                                ),
                                new WaitCommand(350),
                                new InstantCommand(() -> robot.claw.autoReleaseLeft()),
                                new WaitCommand(500),
                                new ParallelCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement3Left),
                                        new TapeDropCommand(robot)
                                )

                        )
                );
                break;
            case RIGHT:
                TrajectorySequence movement1Right = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-43.10, -64.51, Math.toRadians(90.00)))
                        .splineTo(new Vector2d(-31.26, -39.09), Math.toRadians(20.00))
                        .splineToConstantHeading(new Vector2d(-34.56, -40.48), Math.toRadians(90.00))
                        .splineToConstantHeading(new Vector2d(-48.32, -46.75), Math.toRadians(117.50))
                        .splineTo(new Vector2d(-48.49, -23.25), Math.toRadians(90.00))
                        .splineTo(new Vector2d(-4.79, -11.58), Math.toRadians(0.00))
                        .splineTo(new Vector2d(36.83, -11.58), Math.toRadians(0.00))
                        .build();

                TrajectorySequence movement2Right = robot.driveSubsystem.trajectorySequenceBuilder(movement1Right.end())
                        .splineToConstantHeading(new Vector2d(50.58, -42.05), Math.toRadians(0.00))
                        .build();

                TrajectorySequence movement3Right = robot.driveSubsystem.trajectorySequenceBuilder(movement2Right.end())
                        .splineTo(new Vector2d(40.83, -42.05), Math.toRadians(0.00))
                        .splineToConstantHeading(new Vector2d(59.11, -61.90), Math.toRadians(0.00))
                        .build();

                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new DriveCommand(robot.driveSubsystem, movement1Right),
                                new WaitCommand(350), //maximize
                                new ParallelCommandGroup(
                                        new OuttakeCommand(robot),
                                        new DriveCommand(robot.driveSubsystem, movement2Right)
                                ),
                                new WaitCommand(350),
                                new InstantCommand(() -> robot.claw.autoReleaseLeft()),
                                new WaitCommand(500),
                                new ParallelCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement3Right),
                                        new TapeDropCommand(robot)
                                )

                        )
                );
                break;
            case MIDDLE:
                TrajectorySequence movement1Middle = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-43.10, -64.51, Math.toRadians(90.00)))
                        .splineToConstantHeading(new Vector2d(-44.66, -33.00), Math.toRadians(90.00))
                        .splineToConstantHeading(new Vector2d(-46.06, -46.40), Math.toRadians(117.50))
                        .splineTo(new Vector2d(-53.19, -25.33), Math.toRadians(90.00))
                        .splineTo(new Vector2d(-4.79, -11.58), Math.toRadians(0.00))
                        .splineTo(new Vector2d(36.83, -11.58), Math.toRadians(0.00))
                        .build();

                TrajectorySequence movement2Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement1Middle.end())
                        .splineToConstantHeading(new Vector2d(50.58, -37.87), Math.toRadians(0.00))
                        .build();

                TrajectorySequence movement3Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement2Middle.end())
                        .splineTo(new Vector2d(40.31, -37.87), Math.toRadians(0.00))
                        .splineToConstantHeading(new Vector2d(59.11, -61.90), Math.toRadians(0.00))
                        .build();

                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new DriveCommand(robot.driveSubsystem, movement1Middle),
                                new WaitCommand(350), //maximize
                                new ParallelCommandGroup(
                                        new OuttakeCommand(robot),
                                        new DriveCommand(robot.driveSubsystem, movement2Middle)
                                ),
                                new WaitCommand(350),
                                new InstantCommand(() -> robot.claw.autoReleaseLeft()),
                                new WaitCommand(500),
                                new ParallelCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement3Middle),
                                        new TapeDropCommand(robot)
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