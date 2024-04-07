package org.firstinspires.ftc.teamcode.Drive.OpModes.Worlds.Blue.Right.Normal;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.Utility.RoadRunner.DriveConstants;
import org.firstinspires.ftc.teamcode.Utility.RoadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.TrajectorySequences.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.DriveCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.LowOuttakeCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.RestCommand;
import org.firstinspires.ftc.teamcode.Utility.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.Vision.Prop.NewBlueRightProcessor;
import org.firstinspires.ftc.teamcode.Utility.Vision.Robot.Wall.BlueWallRobotScan;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class BlueRightTrussParkRight extends OpMode {
    private VisionPortal visionPortal;
    private NewBlueRightProcessor colorMassDetectionProcessor;
    private BlueWallRobotScan robotProcessor;

    private RobotHardware robot;
    private ElapsedTime time_since_start;
    private double loop;
    boolean robotSensed;

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

        colorMassDetectionProcessor = new NewBlueRightProcessor();
        robotProcessor = new BlueWallRobotScan();

        colorMassDetectionProcessor.setDetectionColor(false); //false is blue, true is red
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .addProcessors(colorMassDetectionProcessor, robotProcessor)
                .build();
    }

    @Override
    public void init_loop() {
        telemetry.addData("Currently Recorded Position: ", colorMassDetectionProcessor.getPropLocation());
        telemetry.addData("Camera State: ", visionPortal.getCameraState());
        CommandScheduler.getInstance().run();
        robot.armSystem.loop();
    }

    @Override
    public void start() {
        time_since_start = new ElapsedTime();

        FtcDashboard.getInstance().startCameraStream(robotProcessor, 30);

        NewBlueRightProcessor.PropPositions recordedPropPosition = colorMassDetectionProcessor.getPropLocation();
        robot.driveSubsystem.setPoseEstimate(new Pose2d(-40.11, 63.48, Math.toRadians(-90.00)));
        switch (recordedPropPosition) {
            case RIGHT:
                TrajectorySequence movement1Right = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, 63.48, Math.toRadians(-90.00)))
                        .splineToConstantHeading(
                                new Vector2d(-51.6, 43.70), Math.toRadians(-90.00),
                                SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToSplineHeading(
                                new Pose2d(-46, 59, Math.toRadians(0.00)),
                                SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToConstantHeading(
                                new Vector2d(9, 59),
                                SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .turn(Math.toRadians(-30))
                        .build();

                TrajectorySequence movement2Right = robot.driveSubsystem.trajectorySequenceBuilder(movement1Right.end())
                        .lineToSplineHeading(
                                new Pose2d(37.72, 59, Math.toRadians(0.00)),
                                SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToConstantHeading(
                                new Vector2d(37.72, 27),
                                SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToConstantHeading(
                                new Vector2d(50, 27),
                                SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence movement3Right = robot.driveSubsystem.trajectorySequenceBuilder(movement2Right.end())
                        .forward(
                                -15,
                                SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence movement4Right = robot.driveSubsystem.trajectorySequenceBuilder(movement3Right.end())
                        .strafeRight(
                                18,
                                SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .forward(
                                23,
                                SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new DriveCommand(robot.driveSubsystem, movement1Right),
                                new WaitUntilCommand(() -> robotSensed || time_since_start.seconds() > 22),
                                new ParallelCommandGroup(
                                        new LowOuttakeCommand(robot),
                                        new DriveCommand(robot.driveSubsystem, movement2Right)
                                ),
                                new WaitCommand(350),
                                new InstantCommand(() -> robot.claw.releaseLeft()),
                                new WaitCommand(350),
                                new DriveCommand(robot.driveSubsystem, movement3Right),
                                new RestCommand(robot),
                                new DriveCommand(robot.driveSubsystem, movement4Right)

                        )
                );
                break;
            case LEFT:
            case UNFOUND:
                TrajectorySequence movement1Left = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, 63.48, Math.toRadians(-90.00)))
                        .splineTo(
                                new Vector2d(-34.99, 37.93), Math.toRadians(-45.00),
                                SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToSplineHeading(
                                new Pose2d(-46, 59, Math.toRadians(0.00)),
                                SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToConstantHeading(
                                new Vector2d(9, 59),
                                SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .turn(Math.toRadians(-30))
                        .build();

                TrajectorySequence movement2Left = robot.driveSubsystem.trajectorySequenceBuilder(movement1Left.end())
                        .lineToSplineHeading(
                                new Pose2d(37.72, 59, Math.toRadians(0.00)),
                                SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToConstantHeading(
                                new Vector2d(37.72, 39.5),
                                SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToConstantHeading(
                                new Vector2d(50, 39.5),
                                SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence movement3Left = robot.driveSubsystem.trajectorySequenceBuilder(movement2Left.end())
                        .forward(
                                -15,
                                SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence movement4Left = robot.driveSubsystem.trajectorySequenceBuilder(movement3Left.end())
                        .strafeRight(
                                47.5,
                                SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .forward(
                                23,
                                SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new DriveCommand(robot.driveSubsystem, movement1Left),
                                new WaitUntilCommand(() -> robotSensed || time_since_start.seconds() > 22),
                                new ParallelCommandGroup(
                                        new LowOuttakeCommand(robot),
                                        new DriveCommand(robot.driveSubsystem, movement2Left)
                                ),
                                new InstantCommand(() -> robot.claw.releaseLeft()),
                                new WaitCommand(350),
                                new DriveCommand(robot.driveSubsystem, movement3Left),
                                new RestCommand(robot),
                                new DriveCommand(robot.driveSubsystem, movement4Left)

                        )
                );
                break;
            case MIDDLE:
                TrajectorySequence movement1Middle = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, 63.48, Math.toRadians(-90.00)))
                        .splineToConstantHeading(
                                new Vector2d(-44.00, 33.00), Math.toRadians(-90.00),
                                SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToSplineHeading(
                                new Pose2d(-46, 59, Math.toRadians(0.00)),
                                SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToConstantHeading(
                                new Vector2d(9, 59),
                                SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .turn(Math.toRadians(-30))
                        .build();

                TrajectorySequence movement2Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement1Middle.end())
                        .lineToSplineHeading(
                                new Pose2d(37.72, 59, Math.toRadians(0.00)),
                                SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToConstantHeading(
                                new Vector2d(37.72, 33.1),
                                SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToConstantHeading(
                                new Vector2d(50.5, 33.1),
                                SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence movement3Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement2Middle.end())
                        .forward(
                                -15,
                                SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence movement4Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement3Middle.end())
                        .strafeRight(
                                22,
                                SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .forward(
                                22,
                                SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new WaitCommand(5000),
                                new DriveCommand(robot.driveSubsystem, movement1Middle),
                                new WaitUntilCommand(() -> robotSensed || time_since_start.seconds() > 22),
                                new ParallelCommandGroup(
                                        new LowOuttakeCommand(robot),
                                        new DriveCommand(robot.driveSubsystem, movement2Middle)
                                ),
                                new WaitCommand(350),
                                new InstantCommand(() -> robot.claw.releaseLeft()),
                                new WaitCommand(350),
                                new DriveCommand(robot.driveSubsystem, movement3Middle),
                                new RestCommand(robot),
                                new DriveCommand(robot.driveSubsystem, movement4Middle)

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
        telemetry.addData("loop ", time - loop);
        telemetry.addData("Time Elapsed: ", time_since_start);
        telemetry.addData("Current Loop Time: ", time - loop);

        robotSensed = (robotProcessor.getSensedBoolean() != BlueWallRobotScan.Sensed.TRUE);

        telemetry.addData("Robot Sensed: ", robotSensed);

        loop = time;

        telemetry.update();
    }

    @Override
    public void stop() {
        visionPortal.close();
        telemetry.addLine("Closed Camera.");
        telemetry.update();
        CommandScheduler.getInstance().reset();
    }
}