package org.firstinspires.ftc.teamcode.common.commandbase.command;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

public class GrabRightCommand extends SequentialCommandGroup {
    public GrabRightCommand(Robot robot) {
        super(
                new InstantCommand(() -> robot.claw.grabRight())
        );
    }
}