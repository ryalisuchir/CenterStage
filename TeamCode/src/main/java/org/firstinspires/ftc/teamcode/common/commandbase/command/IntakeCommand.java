package org.firstinspires.ftc.teamcode.common.commandbase.command;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

//description: ready to intake
public class IntakeCommand extends SequentialCommandGroup {
    public IntakeCommand(Robot robot) {
        super(
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.a.armIntake()),
                        new InstantCommand(() -> robot.angle.rest())
                ),
                new WaitCommand(200),
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.angle.intake()),
                        new InstantCommand(() -> robot.claw.releaseBoth())
                )
        );
    }
}