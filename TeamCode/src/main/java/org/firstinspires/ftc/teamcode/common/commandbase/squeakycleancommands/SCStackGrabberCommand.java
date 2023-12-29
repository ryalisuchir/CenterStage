package org.firstinspires.ftc.teamcode.common.commandbase.squeakycleancommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

//description: ready to intake
public class SCStackGrabberCommand extends SequentialCommandGroup {
    public SCStackGrabberCommand(Robot robot) {
        super(
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.angle.intake()),
                        new InstantCommand(() -> robot.sca.setPosition(10, 0.5))
                )
        );
    }
}