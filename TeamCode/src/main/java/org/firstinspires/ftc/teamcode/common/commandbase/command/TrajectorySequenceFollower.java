package org.firstinspires.ftc.teamcode.common.commandbase.command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.Drive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class TrajectorySequenceFollower extends CommandBase {

    private final Drive drive;
    private final TrajectorySequence trajectory;

    public TrajectorySequenceFollower(Drive drive, TrajectorySequence trajectory) {
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