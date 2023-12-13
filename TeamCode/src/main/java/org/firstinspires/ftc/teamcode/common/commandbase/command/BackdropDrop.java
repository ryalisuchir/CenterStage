package org.firstinspires.ftc.teamcode.common.commandbase.command;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

//command to change angle of arm and then open right claw
public class BackdropDrop extends SequentialCommandGroup {
    public BackdropDrop(Robot robot) {
        super(
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.a.armOuttake()),
                        new InstantCommand(() -> robot.angle.outtake())
                )
        );
    }
}