package org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Utility.Hardware.Globals;
import org.firstinspires.ftc.teamcode.Utility.Hardware.RobotHardware;

public class IntakeCommand extends SequentialCommandGroup {
    public IntakeCommand(RobotHardware robot) {
        super(
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.armSystem.setPosition(50)),
                        new InstantCommand(() -> robot.angleOfArm.intake()),
                        new InstantCommand(() -> robot.claw.grabBoth())
                ),
                new WaitCommand(500),
                new InstantCommand(() -> robot.claw.releaseBoth()),
                new InstantCommand(() -> robot.armSystem.armIntake())
        );
        Globals.startIntake();
    }
}