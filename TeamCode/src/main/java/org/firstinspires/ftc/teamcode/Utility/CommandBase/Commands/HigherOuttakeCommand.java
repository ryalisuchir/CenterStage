package org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Utility.Hardware.Globals;
import org.firstinspires.ftc.teamcode.Utility.Hardware.RobotHardware;

public class HigherOuttakeCommand extends ParallelCommandGroup {
    public HigherOuttakeCommand(RobotHardware robot) {
        super(
                new InstantCommand(() -> robot.armSystem.armOuttake()),
                new InstantCommand(() -> robot.slidesSubsystem.HIGHouttake()),
                new InstantCommand(() -> robot.angleOfArm.outtake())
        );
        Globals.startOuttake();
    }
}