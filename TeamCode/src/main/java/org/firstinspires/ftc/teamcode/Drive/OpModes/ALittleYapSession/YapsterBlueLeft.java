package org.firstinspires.ftc.teamcode.Drive.OpModes.ALittleYapSession;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.outoftheboxrobotics.photoncore.Photon;
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
import org.firstinspires.ftc.teamcode.Utility.Vision.BlueLeftProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

import java.util.List;

@Autonomous
public class YapsterBlueLeft extends OpMode {
    private VisionPortal visionPortal;
    private BlueLeftProcessor colorMassDetectionProcessor;

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
//        robot.armSystem.loop();
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
                                new Vector2d(27.8, 43.31), Math.toRadians(270.00)
                        )
                        .splineToConstantHeading(
                                new Vector2d(27.52, 56.52), Math.toRadians(270.00)
                        )
                        .build();

                TrajectorySequence movement2Left = robot.driveSubsystem.trajectorySequenceBuilder(movement1Left.end())
                        .splineToSplineHeading(
                                new Pose2d(52.30, 41.00, Math.toRadians(0.00)), Math.toRadians(0.00)
                        )
                        .build();

                TrajectorySequence movement3Left = robot.driveSubsystem.trajectorySequenceBuilder(movement2Left.end())
                        .setReversed(true)
                        .splineToConstantHeading(
                                new Vector2d(10.84, 63), Math.toRadians(180.00)
                        )
                        .splineToConstantHeading(
                                new Vector2d(-50, 63), Math.toRadians(180.00)
                        )
                        .splineToConstantHeading(
                                new Vector2d(-62, 38), Math.toRadians(180)
                        )
                        .build();

                TrajectorySequence movement4Left = robot.driveSubsystem.trajectorySequenceBuilder(movement3Left.end())
                        .splineToConstantHeading(
                                new Vector2d(-30.01, 62.33), Math.toRadians(0.00),
                                SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .splineToConstantHeading(
                                new Vector2d(37.93, 60.94), Math.toRadians(0.00),
                                SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .splineToConstantHeading(
                                new Vector2d(38.33, 37.14), Math.toRadians(0.00),
                                SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence movement5Left = robot.driveSubsystem.trajectorySequenceBuilder(movement4Left.end())
                        .lineToConstantHeading(
                                new Vector2d(50, 36.74),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence movement6Left = robot.driveSubsystem.trajectorySequenceBuilder(movement5Left.end())
                        .lineToConstantHeading(new Vector2d(41.69, 37.14))
                        .build();

                TrajectorySequence movement7Left = robot.driveSubsystem.trajectorySequenceBuilder(movement6Left.end())
                        .splineToConstantHeading(
                                new Vector2d(56.15, 61.30), Math.toRadians(0.00),
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
                                new ParallelCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement3Left),
                                        new StackCommand(robot)
                                ),
                                new WaitCommand(750),
                                new InstantCommand(() -> robot.claw.grabBoth()),
                                new WaitCommand(750),
                                new DriveCommand(robot.driveSubsystem, movement4Left),
                                new SuperHighOuttakeCommand(robot),
                                new DriveCommand(robot.driveSubsystem, movement5Left),
                                new WaitCommand(750),
                                new TwoPixelDropCommand(robot),
                                new WaitCommand(750),
                                new DriveCommand(robot.driveSubsystem, movement6Left),
                                new ParallelCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement7Left),
                                        new RestCommand(robot)
                                )

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