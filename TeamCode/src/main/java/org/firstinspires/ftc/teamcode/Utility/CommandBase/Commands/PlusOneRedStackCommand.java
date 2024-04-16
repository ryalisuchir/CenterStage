package org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Utility.Hardware.Globals;
import org.firstinspires.ftc.teamcode.Utility.Hardware.RobotHardware;

public class PlusOneRedStackCommand extends SequentialCommandGroup {
    public PlusOneRedStackCommand(RobotHardware robot) {
        super(
                new ParallelCommandGroup(new InstantCommand(() -> robot.slidesSubsystem.intake()),
                        new InstantCommand(() -> robot.angleOfArm.plusOneStack())),
                new WaitCommand(600),
                new ParallelCommandGroup(new InstantCommand(() -> robot.armSystem.armCoast()),
                        new InstantCommand(() -> robot.claw.releaseLeft()))
        );
        Globals.startIntake();
    }
}