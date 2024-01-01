package org.firstinspires.ftc.teamcode.drive.tests.hardware;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.common.commandbase.command.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
@Disabled
public class RRTest extends OpMode {
    private Robot robot;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        robot = new Robot(hardwareMap);
        CommandScheduler.getInstance().registerSubsystem(robot.driveSubsystem);
    }

    @Override
    public void start() {
        robot.driveSubsystem.setPoseEstimate(new Pose2d(-39.61, 67.30, Math.toRadians(270.00)));
        TrajectorySequence testRR = robot.driveSubsystem
                .trajectorySequenceBuilder(new Pose2d(-39.61, 67.30, Math.toRadians(270.00)))
                .splineTo(new Vector2d(-32.30, 32.30), Math.toRadians(270.00))
                .build();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new WaitCommand(500),
                        new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequencenotAsync(testRR))
                )
        );
    }
    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        robot.driveSubsystem.update();
    }

    @Override
    public void stop() {
        CommandScheduler.getInstance().reset();
    }
}
