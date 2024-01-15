package org.firstinspires.ftc.teamcode.Drive.OpModes.AutonomousPaths.RedRight;

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
public class RedRightLeftAuto extends OpMode {
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
        telemetry.addData("Successful: ", "Ready for RedRight - Left - (Backdrop Side)");
        telemetry.addData("Ready to Run: ", "2 pixel autonomous. All subsystems initialized.");
        CommandScheduler.getInstance().run();
        robot.armSystem.loop();
    }

    @Override
    public void start() {
        time_since_start = new ElapsedTime();
        robot.driveSubsystem.setPoseEstimate(new Pose2d(17.85, -65.56, Math.toRadians(90.00)));

        TrajectorySequence backdropLeft = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(17.85, -65.56, Math.toRadians(90.00)))
                .splineTo(
                        new Vector2d(49.54, -30.04), Math.toRadians(0.00),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        TrajectorySequence tapeLeft = robot.driveSubsystem.trajectorySequenceBuilder(backdropLeft.end())
                .lineToSplineHeading(
                        new Pose2d(10.36, -38.39, Math.toRadians(0.00)),
                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        TrajectorySequence parkLeft = robot.driveSubsystem.trajectorySequenceBuilder(tapeLeft.end())
                .lineToConstantHeading(new Vector2d(19.24, -38.22))
                .lineToConstantHeading(new Vector2d(34.56, -62.42))
                .lineToSplineHeading(
                        new Pose2d(62.95, -62.25, Math.toRadians(0.00)),
                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequenceNotAsync(backdropLeft)),
                                new OuttakeCommand(robot)
                        ),
                        new WaitCommand(3500),
                        new InstantCommand(() -> robot.claw.autoReleaseLeft()),
                        new WaitCommand(350),
                        new ParallelCommandGroup(
                                new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequenceNotAsync(tapeLeft)),
                                new TapeDropCommand(robot)
                        ),
                        new WaitCommand(350),
                        new InstantCommand(() -> robot.claw.releaseRight()),
                        new WaitCommand(1000),
                        new RestCommand(robot),
                        new WaitCommand(350),
                        new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequenceNotAsync(parkLeft))

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