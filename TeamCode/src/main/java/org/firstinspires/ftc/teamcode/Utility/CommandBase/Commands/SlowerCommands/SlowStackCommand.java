package org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.SlowerCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.Utility.Hardware.Globals;
import org.firstinspires.ftc.teamcode.Utility.Hardware.RobotHardware;

public class SlowStackCommand extends ParallelCommandGroup {
    public SlowStackCommand(RobotHardware robot) {
        super(
                new InstantCommand(() -> robot.slidesSubsystem.intake()),
                new InstantCommand(() -> robot.slowArmSystem.setPosition(50)),
                new InstantCommand(() -> robot.angleOfArm.stack()),
                new InstantCommand(() -> robot.claw.releaseLeft())
        );
        Globals.startIntake();
    }
}