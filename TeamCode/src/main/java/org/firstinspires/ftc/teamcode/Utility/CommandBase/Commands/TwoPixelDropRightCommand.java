package org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Utility.Hardware.RobotHardware;

public class TwoPixelDropRightCommand extends SequentialCommandGroup {
    public TwoPixelDropRightCommand(RobotHardware robot) {
        super(
                new InstantCommand(() -> robot.claw.smallReleaseRight())

        );
    }
}