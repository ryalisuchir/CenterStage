package org.firstinspires.ftc.teamcode.common.commandbase.squeakycleancommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

//description: after pickup, before outtake (while going across the mat)
public class SCRestCommand extends SequentialCommandGroup {
    public SCRestCommand(Robot robot) {
        super(
                new InstantCommand(() -> robot.sca.setPosition(0, 0.3)),
                new InstantCommand(() -> robot.angle.outtake())
        );
    }
}