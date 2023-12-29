package org.firstinspires.ftc.teamcode.common.commandbase.squeakycleancommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

//description: drops the pixel on the tape during auto. just sets up position.
public class SCTapeDropperCommand extends SequentialCommandGroup {
    public SCTapeDropperCommand(Robot robot) {
        super(
                new InstantCommand(() -> robot.sca.setPosition(5, 0.5)),
                new InstantCommand(() -> robot.angle.tapeDrop())
        );
    }
}