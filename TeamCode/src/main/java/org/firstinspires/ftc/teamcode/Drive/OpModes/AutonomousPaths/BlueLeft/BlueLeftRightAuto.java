package org.firstinspires.ftc.teamcode.Drive.OpModes.AutonomousPaths.BlueLeft;

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
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.DriveCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.OuttakeCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.RestCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.TapeDropCommand;
import org.firstinspires.ftc.teamcode.Utility.Hardware.RobotHardware;

@Autonomous
public class BlueLeftRightAuto extends OpMode {
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
        telemetry.addData("Successful: ", "Ready for BlueLeft - Right - (Backdrop Side)");
        telemetry.addData("Ready to Run: ", "2 pixel autonomous. All subsystems initialized.");
        CommandScheduler.getInstance().run();
        robot.armSystem.loop();
    }

    @Override
    public void start() {
        time_since_start = new ElapsedTime();
        robot.driveSubsystem.setPoseEstimate(new Pose2d(15.76, 63.99, Math.toRadians(-90.00)));

        TrajectorySequence backdropRight = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(15.76, 63.99, Math.toRadians(-90.00)))
                .splineTo(
                        new Vector2d(49.19, 29.69), Math.toRadians(0.00),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        TrajectorySequence tapeRight = robot.driveSubsystem.trajectorySequenceBuilder(backdropRight.end())
                .lineToSplineHeading(
                        new Pose2d(11.41, 38.22, Math.toRadians(0.00)),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        TrajectorySequence parkRight = robot.driveSubsystem.trajectorySequenceBuilder(tapeRight.end())
                .lineToConstantHeading(new Vector2d(20.81, 37.7))
                .lineToConstantHeading(new Vector2d(35.78, 63.29))
                .lineToConstantHeading(
                        new Vector2d(58.94, 63.12),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new DriveCommand(robot.driveSubsystem, backdropRight),
                                //new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequenceNotAsync(backdropRight)),
                                new OuttakeCommand(robot)
                        ),
                        new WaitCommand(350),
                        new InstantCommand(() -> robot.claw.autoReleaseLeft()),
                        new WaitCommand(350),
                        new ParallelCommandGroup(
                                new DriveCommand(robot.driveSubsystem, tapeRight),
                                //new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequenceNotAsync(tapeRight)),
                                new TapeDropCommand(robot)
                        ),
                        new WaitCommand(350),
                        new InstantCommand(() -> robot.claw.releaseRight()),
                        new WaitCommand(1000),
                        new RestCommand(robot),
                        new WaitCommand(350),
                        new DriveCommand(robot.driveSubsystem, parkRight)
                        //new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequenceNotAsync(parkRight))

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