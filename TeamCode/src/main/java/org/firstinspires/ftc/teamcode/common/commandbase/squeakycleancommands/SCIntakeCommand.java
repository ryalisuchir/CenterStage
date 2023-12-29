package org.firstinspires.ftc.teamcode.common.commandbase.squeakycleancommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

//description: ready to intake
public class SCIntakeCommand extends SequentialCommandGroup {
    public SCIntakeCommand(Robot robot) {
        super(
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.sca.setPosition(5,0.3)),
                        new InstantCommand(() -> robot.angle.intake()),
                        new InstantCommand(() -> robot.claw.grabBoth())
                ),
                new WaitCommand(1000),
                new InstantCommand(() -> robot.sca.setPosition(0,0.3)),
                new InstantCommand(() -> robot.claw.releaseBoth())
        );
    }
}