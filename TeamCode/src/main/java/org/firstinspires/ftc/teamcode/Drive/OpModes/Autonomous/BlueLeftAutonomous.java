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

import org.firstinspires.ftc.teamcode.Drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.TrajectorySequences.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.OuttakeCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.RestCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.StackCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.TapeDropCommand;
import org.firstinspires.ftc.teamcode.Utility.Hardware.RobotHardware;

@Autonomous
@Config
public class BlueLeftAutonomous extends OpMode {
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

    }
    @Override
    public void init_loop() {
        telemetry.addData("Successful: ", "Ready for BlueLeft (Backdrop Side)");
        telemetry.addData("Ready to Run: ", "2 pixel autonomous. All subsystems initialized.");
        CommandScheduler.getInstance().run();
        robot.armSystem.loop();
    }

    @Override
    public void start() {
        time_since_start = new ElapsedTime();
        robot.driveSubsystem.setPoseEstimate(new Pose2d(15.76, 63.99, Math.toRadians(-90.00)));

        TrajectorySequence backdropMiddle = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(15.76, 63.99, Math.toRadians(-90.00)))
                .splineTo(
                        new Vector2d(49.54, 35.61), Math.toRadians(0.00),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        TrajectorySequence tapeMiddle = robot.driveSubsystem.trajectorySequenceBuilder(backdropMiddle.end())
                .lineToSplineHeading(
                        new Pose2d(23.94, 24.99, Math.toRadians(0.00)),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        TrajectorySequence speedToStackMiddle = robot.driveSubsystem.trajectorySequenceBuilder(tapeMiddle.end())
                .lineToSplineHeading(
                        new Pose2d(23.94, 59.81, Math.toRadians(0.00)),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .lineToSplineHeading(
                        new Pose2d(-52.15, 59.46, Math.toRadians(0.00)),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .lineToSplineHeading(
                        new Pose2d(-52.50, 11.75, Math.toRadians(0.00)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        TrajectorySequence slowGrabMiddle = robot.driveSubsystem.trajectorySequenceBuilder(speedToStackMiddle.end())
                .lineToConstantHeading(
                        new Vector2d(-58.59, 11.58),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        TrajectorySequence speedToBackboardMiddle = robot.driveSubsystem.trajectorySequenceBuilder(slowGrabMiddle.end())
                .splineTo(
                        new Vector2d(31.95, 11.93), Math.toRadians(0.93),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .lineToConstantHeading(
                        new Vector2d(32.30, 41.35),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        TrajectorySequence slowBackboardMiddle = robot.driveSubsystem.trajectorySequenceBuilder(speedToBackboardMiddle.end())
                .lineToConstantHeading(
                        new Vector2d(49.54, 40.83),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        TrajectorySequence parkMiddle = robot.driveSubsystem.trajectorySequenceBuilder(slowBackboardMiddle.end())
                .lineToSplineHeading(
                        new Pose2d(49.54, 61.38, Math.toRadians(0.00)),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .splineTo(
                        new Vector2d(63.29, 61.38), Math.toRadians(0.00),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequenceNotAsync(backdropMiddle)),
                                new OuttakeCommand(robot)
                        ),
                        new InstantCommand(() -> robot.claw.autoReleaseLeft()),
                        new WaitCommand(350),
                        new InstantCommand(() -> robot.angleOfArm.backBoardPress()),
                        new WaitCommand(350),
                        new ParallelCommandGroup(
                                new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequenceNotAsync(tapeMiddle)),
                                new TapeDropCommand(robot)
                        ),
                        new WaitCommand(350),
                        new InstantCommand(() -> robot.claw.releaseRight()),
                        new WaitCommand(350),
                        new RestCommand(robot),
                        new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequenceNotAsync(speedToStackMiddle)),
                        new ParallelCommandGroup(
                                new StackCommand(robot),
                                new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequenceNotAsync(slowGrabMiddle))
                        ),
                        new WaitCommand(350),
                        new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequenceNotAsync(speedToBackboardMiddle)),
                        new WaitCommand(350),
                        new OuttakeCommand(robot),
                        new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequenceNotAsync(slowBackboardMiddle)),
                        new WaitCommand(350),
                        new InstantCommand(() -> robot.claw.autoReleaseLeft()),
                        new WaitCommand(350),
                        new RestCommand(robot),
                        new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequenceNotAsync(parkMiddle))

                )
        );

    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        robot.armSystem.loop();
        robot.driveSubsystem.update();
        robot.slidesSubsystem.loop();

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
        CommandScheduler.getInstance().reset();
    }
}