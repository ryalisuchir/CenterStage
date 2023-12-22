package org.firstinspires.ftc.teamcode.common.commandbase.command;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

//description: after pickup, before outtake (while going across the mat)
public class RestCommand extends SequentialCommandGroup {
    public RestCommand(Robot robot) {
        super(
                new InstantCommand(() -> robot.a.armCoast()),
                new InstantCommand(() -> robot.angle.outtake())
        );
    }
}