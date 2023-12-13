package org.firstinspires.ftc.teamcode.common.commandbase.command;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

//command to intake and go to resting position
public class Intake extends SequentialCommandGroup {
    public Intake(Robot robot) {
        super(
                new InstantCommand(() -> robot.slides.autoIntake()),
                new InstantCommand(() -> robot.a.armIntake()),
                new WaitCommand(350),
                new InstantCommand(() -> robot.claw.grabBoth()),
                new WaitCommand(350)
        );
    }
}