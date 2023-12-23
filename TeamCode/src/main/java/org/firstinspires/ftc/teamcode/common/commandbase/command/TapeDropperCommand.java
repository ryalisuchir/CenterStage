package org.firstinspires.ftc.teamcode.common.commandbase.command;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

//description: drops the pixel on the tape during auto. just sets up position.
public class TapeDropperCommand extends SequentialCommandGroup {
    public TapeDropperCommand(Robot robot) {
        super(
                new InstantCommand(() -> robot.a.armTapeDrop()),
                new InstantCommand(() -> robot.angle.tapeDrop())
        );
    }
}