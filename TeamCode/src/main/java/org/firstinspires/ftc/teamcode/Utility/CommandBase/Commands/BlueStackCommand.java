package org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Utility.Hardware.Globals;
import org.firstinspires.ftc.teamcode.Utility.Hardware.RobotHardware;

public class BlueStackCommand extends SequentialCommandGroup {
    public BlueStackCommand(RobotHardware robot) {
        super(
                new ParallelCommandGroup(new InstantCommand(() -> robot.slidesSubsystem.intake()),
                        new InstantCommand(() -> robot.angleOfArm.lolStack())),
                new WaitCommand(250),
                new ParallelCommandGroup(new InstantCommand(() -> robot.armSystem.armCoast()),
                        new InstantCommand(() -> robot.claw.releaseRight()))
        );
        Globals.startIntake();
    }
    public BlueStackCommand(RobotHardware robot, double dumpy) {
        super(
                new ParallelCommandGroup(new InstantCommand(() -> robot.slidesSubsystem.intake()),
                        new InstantCommand(() -> robot.angleOfArm.customAngle(dumpy))),
                new WaitCommand(250),
                new ParallelCommandGroup(new InstantCommand(() -> robot.armSystem.armCoast()),
                        new InstantCommand(() -> robot.claw.releaseRight()))
        );
        Globals.startIntake();
    }
}
//robot.armSystem.armCoast()),