package org.firstinspires.ftc.teamcode.Drive.OpModes.States.AdvancedAutonomous60;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.lynx.LynxModule;
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
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.StackCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.SuperHighOuttakeCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.TwoPixelDropCommand;
import org.firstinspires.ftc.teamcode.Utility.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.Vision.Prop.ANewBlueProcessor;
import org.firstinspires.ftc.teamcode.Utility.Vision.Prop.BlueLeftProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

import java.util.List;

@Autonomous
public class BlueLeft60 extends OpMode {
    private VisionPortal visionPortal;
    private ANewBlueProcessor colorMassDetectionProcessor;

    private RobotHardware robot;
    private ElapsedTime time_since_start;
    private double loop;
    List<LynxModule> allHubs;
    @Override
    public void init() {
        allHubs = hardwareMap.getAll(LynxModule.class);
        CommandScheduler.getInstance().reset();
        robot = new RobotHardware(hardwareMap);

        CommandScheduler.getInstance().registerSubsystem(robot.armSystem);
        CommandScheduler.getInstance().registerSubsystem(robot.claw);
        CommandScheduler.getInstance().registerSubsystem(robot.angleOfArm);
        CommandScheduler.getInstance().registerSubsystem(robot.driveSubsystem);

        telemetry.addData("Not Ready: ", "Not able to proceed to camera detection... Restart robot now.");
        telemetry.update();

        robot.claw.grabBoth();
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        Scalar lower = new Scalar(107.7, 143.1, 38.3);
        Scalar upper = new Scalar(119, 229.5, 133.3);
        double minArea = 100;

        colorMassDetectionProcessor = new ANewBlueProcessor(
                lower,
                upper,
                () -> minArea,
                () -> 213,
                () -> 426
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
//        robot.armSystem.loop();
    }

    @Override
    public void start() {
        time_since_start = new ElapsedTime();
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.stopLiveView();
            visionPortal.stopStreaming();
        }

        ANewBlueProcessor.PropPositions recordedPropPosition = colorMassDetectionProcessor.getRecordedPropPosition();
        robot.driveSubsystem.setPoseEstimate(new Pose2d(16.14, 63.32, Math.toRadians(-90.00)));
        switch (recordedPropPosition) {
            case LEFT:
                TrajectorySequence movement1Left = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(16.14, 63.32, Math.toRadians(-90.00)))
                        .splineToConstantHeading(
                                new Vector2d(27.8, 43.31), Math.toRadians(270.00)
                        )
                        .splineToConstantHeading(
                                new Vector2d(27.52, 56.52), Math.toRadians(270.00)
                        )
                        .build();

                TrajectorySequence movement2Left = robot.driveSubsystem.trajectorySequenceBuilder(movement1Left.end())
                        .splineToSplineHeading(
                                new Pose2d(52.30, 41.00, Math.toRadians(0.00)), Math.toRadians(0.00),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build(); //raise slides with this motion

                TrajectorySequence movement3Left = robot.driveSubsystem.trajectorySequenceBuilder(movement2Left.end())
                        .setReversed(true)
                        .splineToConstantHeading(
                                new Vector2d(10.84, 63), Math.toRadians(180.00)
                        )
                        .splineToConstantHeading(
                                new Vector2d(-50, 63), Math.toRadians(180.00)
                        )
                        .splineToConstantHeading(
                                new Vector2d(-57, 38), Math.toRadians(180)
                        )
                        .forward(-5,
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build(); //get ready to pick up a pixel with this motion

                TrajectorySequence movement4Left = robot.driveSubsystem.trajectorySequenceBuilder(movement3Left.end())
                        .setReversed(false)
                        .splineToConstantHeading(
                                new Vector2d(-30, 60), Math.toRadians(0)
                        )
                        .splineToConstantHeading(
                                new Vector2d(31, 60), Math.toRadians(0)
                        )
                        .splineToConstantHeading(
                                new Vector2d(39, 37), Math.toRadians(0)
                        )
                        .build(); //get back to board with this motion

                TrajectorySequence movement5Left = robot.driveSubsystem.trajectorySequenceBuilder(movement4Left.end())
                        .splineToConstantHeading(
                                new Vector2d(50, 37), Math.toRadians(0.00),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build(); //lift slides with this motion

                TrajectorySequence movement6Left = robot.driveSubsystem.trajectorySequenceBuilder(movement5Left.end())
                        .setReversed(true)
                        .splineToConstantHeading(new Vector2d(37, 37), Math.toRadians(0.00))
                        .build(); //get to rest position with this motion

                TrajectorySequence movement7Left = robot.driveSubsystem.trajectorySequenceBuilder(movement6Left.end())
                        .splineToConstantHeading(
                                new Vector2d(56.15, 61.30), Math.toRadians(0.00)
                        )
                        .build();

                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new DriveCommand(robot.driveSubsystem, movement1Left),
                                new ParallelCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement2Left),
                                        new OuttakeCommand(robot)
                                ),
                                new WaitCommand(500),
                                new InstantCommand(() -> robot.claw.releaseLeft()),
                                new WaitCommand(500),
                                new ParallelCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement3Left),
                                        new StackCommand(robot)
                                ),
                                new WaitCommand(750),
                                new InstantCommand(() -> robot.claw.grabBoth()),
                                new WaitCommand(750),
                                new DriveCommand(robot.driveSubsystem, movement4Left),
                                new ParallelCommandGroup(
                                        new SuperHighOuttakeCommand(robot),
                                        new DriveCommand(robot.driveSubsystem, movement5Left)
                                ),
                                new WaitCommand(750),
                                new TwoPixelDropCommand(robot),
                                new WaitCommand(350),
                                new ParallelCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement6Left),
                                        new RestCommand(robot)
                                ),
                                new DriveCommand(robot.driveSubsystem, movement7Left)
                        )
                );

                break;
            case RIGHT:
            case UNFOUND:
                //do stuff
                break;
            case MIDDLE:
                //do stuff
                break;
        }
    }

    @Override
    public void loop() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        CommandScheduler.getInstance().run();
        robot.armSystem.loop();
        robot.slidesSubsystem.loop();
        robot.driveSubsystem.update();

        double time = System.currentTimeMillis();
        telemetry.addData("Time Elapsed: ", time_since_start);
        telemetry.addData("Current Loop Time: ", time - loop);

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