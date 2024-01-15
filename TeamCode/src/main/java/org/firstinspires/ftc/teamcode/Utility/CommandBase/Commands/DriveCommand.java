package org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.TrajectorySequences.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {
    private final DriveSubsystem drive;
    private final TrajectorySequence trajectory;

    public DriveCommand(DriveSubsystem drive, TrajectorySequence trajectory) {
        this.drive = drive;
        this.trajectory = trajectory;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.followTrajectorySequence(trajectory);
    }

    @Override
    public void execute() {
        drive.update();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            drive.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return Thread.currentThread().isInterrupted() || !drive.isBusy();
    }
}