package org.firstinspires.ftc.teamcode.Drive.OpModes.AutonomousPaths.RedLeft;

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
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.TapeDropCommand;
import org.firstinspires.ftc.teamcode.Utility.Hardware.RobotHardware;

@Autonomous
public class RedLeftRightAuto extends OpMode {
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
        CommandScheduler.getInstance().registerSubsystem(robot.slidesSubsystem);
        CommandScheduler.getInstance().registerSubsystem(robot.driveSubsystem);

        telemetry.update();

        robot.claw.grabBoth();

    }
    @Override
    public void init_loop() {
        telemetry.addData("Successful: ", "Ready for RedRight - Right - (Backdrop Side)");
        telemetry.addData("Ready to Run: ", "2 pixel autonomous. All subsystems initialized.");
        CommandScheduler.getInstance().run();
        robot.armSystem.loop();
    }

    @Override
    public void start() {
        time_since_start = new ElapsedTime();
        robot.driveSubsystem.setPoseEstimate(new Pose2d(-39.96, -63.99, Math.toRadians(90.00)));

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
                        new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequenceNotAsync(tapeRight)),
                        new WaitCommand(350),
                        new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequenceNotAsync(toWaitRight)),
                        new WaitCommand(350), //this can be changed to maximize auto time!
                        new ParallelCommandGroup(
                                new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequenceNotAsync(toBackBoardRight)),
                                new OuttakeCommand(robot)
                        ),
                        new WaitCommand(350),
                        new InstantCommand(() -> robot.claw.autoReleaseLeft()),
                        new WaitCommand(1000),
                        new ParallelCommandGroup(
                                new RestCommand(robot),
                                new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequenceNotAsync(toParkRight))
                        )
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