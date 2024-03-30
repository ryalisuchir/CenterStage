package org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Utility.Hardware.Globals;
import org.firstinspires.ftc.teamcode.Utility.Hardware.RobotHardware;

public class SecondOuttakeCommand extends SequentialCommandGroup {
    public SecondOuttakeCommand(RobotHardware robot) {
        super(
                new InstantCommand(() -> robot.angleOfArm.outtake()),
                new InstantCommand(() -> robot.armSystem.armOuttake()),
                new InstantCommand(() -> robot.slidesSubsystem.superHighOuttakeCommand())
        );
        Globals.startOuttake();
    }
}