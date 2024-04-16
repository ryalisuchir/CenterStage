package org.firstinspires.ftc.teamcode.Drive.OpModes.Worlds.Red.Left.PlusOne;

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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.Utility.RoadRunner.DriveConstants;
import org.firstinspires.ftc.teamcode.Utility.RoadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.TrajectorySequences.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.DriveCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.LowOuttakeCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.PlusOneRedStackCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.RestCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.SecondOuttakeCommand;
import org.firstinspires.ftc.teamcode.Utility.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.Vision.Prop.NewRedLeftProcessor;
import org.firstinspires.ftc.teamcode.Utility.Vision.Robot.Wall.RedWallRobotScan;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
@Disabled
public class PlusOneRedLeftNoPark extends OpMode {
    private VisionPortal visionPortal;
    private NewRedLeftProcessor colorMassDetectionProcessor;
    private RedWallRobotScan robotProcessor;
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

        colorMassDetectionProcessor = new NewRedLeftProcessor();
        robotProcessor = new RedWallRobotScan();

        colorMassDetectionProcessor.setDetectionColor(true); //false is blue, true is red
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

        NewRedLeftProcessor.PropPositions recordedPropPosition = colorMassDetectionProcessor.getPropLocation();
        robot.driveSubsystem.setPoseEstimate(new Pose2d(-40.11, -63.48, Math.toRadians(90.00)));

        switch (recordedPropPosition) {
            case RIGHT:
                TrajectorySequence movement1Right = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, -63.48, Math.toRadians(90.00)))
                        .splineToLinearHeading(new Pose2d(-32.31, -37.58, Math.toRadians(45)), Math.toRadians(45))
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-53, -37.5, Math.toRadians(0.00)), Math.toRadians(180))
                        .build();

                TrajectorySequence extraBack = robot.driveSubsystem.trajectorySequenceBuilder(movement1Right.end())
                        .setReversed(false)
                        .lineToConstantHeading(
                                new Vector2d(-58.5, -37.5),
                                SampleMecanumDrive.getVelocityConstraint(7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence movement2Right = robot.driveSubsystem.trajectorySequenceBuilder(extraBack.end())
                        .lineToConstantHeading(new Vector2d(-52, -37))
                        .splineToConstantHeading(new Vector2d(-32.14, -59.36), Math.toRadians(0.00))
                        .lineToSplineHeading(new Pose2d(4.04, -59.36, Math.toRadians(0.00)))
                        .turn(Math.toRadians(30))
                        .build();

                TrajectorySequence movement3Right = robot.driveSubsystem.trajectorySequenceBuilder(movement2Right.end())
                        .lineToSplineHeading(new Pose2d(30, -59.36, Math.toRadians(0.00)))
                        .splineToConstantHeading(
                                new Vector2d(50, -35), Math.toRadians(0.00)
                        )
                        .build();

                TrajectorySequence movement4Right = robot.driveSubsystem.trajectorySequenceBuilder(movement3Right.end())
                        .setTangent(-180)
                        .splineToConstantHeading(
                                new Vector2d(
                                        48.5, -30), Math.toRadians(0),
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence movement5Right = robot.driveSubsystem.trajectorySequenceBuilder(movement4Right.end())
                        .lineToConstantHeading(new Vector2d(43.5, -30))
                        .build();

                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new DriveCommand(robot.driveSubsystem, movement1Right),
                                new PlusOneRedStackCommand(robot),
                                new DriveCommand(robot.driveSubsystem, extraBack),
                                new WaitCommand(500),
                                new InstantCommand(() -> robot.claw.grabBoth()),
                                new WaitCommand(350),
                                new DriveCommand(robot.driveSubsystem, movement2Right),
                                new WaitUntilCommand(() -> robotSensed || time_since_start.seconds() > 23),
                                new ParallelCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement3Right),
                                        new LowOuttakeCommand(robot)
                                ),
                                new WaitCommand(350),
                                new InstantCommand(() -> robot.claw.releaseRight()),
                                new WaitCommand(350),
                                new ParallelCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement4Right),
                                        new SecondOuttakeCommand(robot)
                                ),
                                new WaitCommand(600),
                                new InstantCommand(() -> robot.claw.releaseLeft()),
                                new WaitCommand(1500),
                                new ParallelCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement5Right),
                                        new RestCommand(robot)
                                )

                        )
                );

                break;
            case LEFT:
            case UNFOUND:
                TrajectorySequence movement1Left = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, -63.48, Math.toRadians(90.00)))
                        .splineToLinearHeading(
                                new Pose2d(-50.00, -43.70, Math.toRadians(90.00)), Math.toRadians(90.00)
                        )
                        .build();

                TrajectorySequence extraBack2 = robot.driveSubsystem.trajectorySequenceBuilder(movement1Left.end())
                        .lineToSplineHeading(
                                new Pose2d(
                                        -55.5, -53, Math.toRadians(0.00)),
                                SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToSplineHeading(
                                new Pose2d(-55.5, -38, Math.toRadians(0.00))
                        )
                        .lineToConstantHeading(
                                new Vector2d(-58.5, -38),
                                SampleMecanumDrive.getVelocityConstraint(7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence movement2Left = robot.driveSubsystem.trajectorySequenceBuilder(extraBack2.end())
                        .lineToConstantHeading(new Vector2d(-56.34, -38))
                        .lineToConstantHeading(new Vector2d(-56.34, -58.5))
                        .lineToConstantHeading(new Vector2d(8, -57))
                        .turn(Math.toRadians(30))
                        .build();

                TrajectorySequence movement3Left = robot.driveSubsystem.trajectorySequenceBuilder(movement2Left.end())
                        .lineToSplineHeading(new Pose2d(20, -57, Math.toRadians(0.00)))
                        .splineToConstantHeading(
                                new Vector2d(49.3, -23), Math.toRadians(0.00),
                                SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence movement4Left = robot.driveSubsystem.trajectorySequenceBuilder(movement3Left.end())
                        .setTangent(-180)
                        .splineToConstantHeading(
                                new Vector2d(
                                        48, -37), Math.toRadians(0),
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence movement5Left = robot.driveSubsystem.trajectorySequenceBuilder(movement4Left.end())
                        .lineToConstantHeading(new Vector2d(43, -37))
                        .build();

                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new DriveCommand(robot.driveSubsystem, movement1Left),
                                new PlusOneRedStackCommand(robot),
                                new DriveCommand(robot.driveSubsystem, extraBack2),
                                new WaitCommand(500),
                                new InstantCommand(() -> robot.claw.grabBoth()),
                                new DriveCommand(robot.driveSubsystem, movement2Left),
                                new WaitUntilCommand(() -> robotSensed || time_since_start.seconds() > 23),
                                new ParallelCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement3Left),
                                        new LowOuttakeCommand(robot)
                                ),
                                new WaitCommand(350),
                                new InstantCommand(() -> robot.claw.releaseRight()),
                                new WaitCommand(350),
                                new ParallelCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement4Left),
                                        new SecondOuttakeCommand(robot)
                                ),
                                new WaitCommand(600),
                                new InstantCommand(() -> robot.claw.releaseLeft()),
                                new WaitCommand(1500),
                                new ParallelCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement5Left),
                                        new RestCommand(robot)
                                )

                        )
                );
                break;
            case MIDDLE:
                TrajectorySequence movement1Middle = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, -63.48, Math.toRadians(90.00)))
                        .splineToConstantHeading(
                                new Vector2d(-44.50, -33.10), Math.toRadians(90.00)
                        )
                        .build();

                TrajectorySequence extraBack3 = robot.driveSubsystem.trajectorySequenceBuilder(movement1Middle.end())
                        .lineToSplineHeading(
                                new Pose2d(-55, -37.5, Math.toRadians(0))
                        )
                        .lineToConstantHeading(
                                new Vector2d(-58.5, -37.5)
                        )
                        .build();

                TrajectorySequence movement2Middle = robot.driveSubsystem.trajectorySequenceBuilder(extraBack3.end())
                        .lineToConstantHeading(new Vector2d(-54, -37.5))
                        .lineToConstantHeading(new Vector2d(-54, -58.5))
                        .lineToConstantHeading(new Vector2d(10, -57))
                        .turn(Math.toRadians(30))
                        .build();

                TrajectorySequence movement3Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement2Middle.end())
                        .lineToSplineHeading(new Pose2d(20, -57, Math.toRadians(0)))
                        .splineToConstantHeading(
                                new Vector2d(52, -30.5), Math.toRadians(0.00),
                                SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence movement4Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement3Middle.end())
                        //                        .lineToConstantHeading(
                        //                                new Vector2d(48, 37),
                        //                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        //                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        //                        )
                        .setTangent(-180)
                        .splineToConstantHeading(
                                new Vector2d(
                                        48.9, -30), Math.toRadians(0),
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence movement5Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement4Middle.end())
                        .lineToConstantHeading(new Vector2d(43.9, -30))
                        .build();

                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new DriveCommand(robot.driveSubsystem, movement1Middle),
                                new PlusOneRedStackCommand(robot),
                                new DriveCommand(robot.driveSubsystem, extraBack3),
                                new WaitCommand(500),
                                new InstantCommand(() -> robot.claw.grabBoth()),
                                new DriveCommand(robot.driveSubsystem, movement2Middle),
                                new WaitUntilCommand(() -> robotSensed || time_since_start.seconds() > 23),
                                new ParallelCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement3Middle),
                                        new LowOuttakeCommand(robot)
                                ),
                                new WaitCommand(350),
                                new InstantCommand(() -> robot.claw.releaseRight()),
                                new WaitCommand(350),
                                new ParallelCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement4Middle),
                                        new SecondOuttakeCommand(robot)
                                ),
                                new WaitCommand(600),
                                new InstantCommand(() -> robot.claw.releaseLeft()),
                                new WaitCommand(1500),
                                new ParallelCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement5Middle),
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
        telemetry.addData("loop ", time - loop);
        telemetry.addData("Time Elapsed: ", time_since_start);
        telemetry.addData("Current Loop Time: ", time - loop);

        robotSensed = (robotProcessor.getSensedBoolean() != RedWallRobotScan.Sensed.TRUE);

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