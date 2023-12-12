package org.firstinspires.ftc.teamcode.common.commandbase.command;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

//command to change angle of arm and then open right claw
public class Dump extends SequentialCommandGroup {
    public Dump(Robot robot) {
        super(
                new InstantCommand(() -> robot.angle.out()),
                new WaitCommand(350),
                new InstantCommand(() -> robot.claw.releaseRight()),
                new WaitCommand(350)
        );
    }
}