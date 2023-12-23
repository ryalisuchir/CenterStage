package org.firstinspires.ftc.teamcode.common.commandbase.command;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

//description: ready to intake
public class FirstStackGrabCommand extends SequentialCommandGroup {
    public FirstStackGrabCommand(Robot robot) {
        super(
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.angle.intake()),
                        new InstantCommand(() -> robot.a.armTapeDrop())
                )
        );
    }
}