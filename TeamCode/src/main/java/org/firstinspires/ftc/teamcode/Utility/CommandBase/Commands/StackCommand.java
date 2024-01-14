package org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Utility.Hardware.Globals;
import org.firstinspires.ftc.teamcode.Utility.Hardware.RobotHardware;

public class StackCommand extends ParallelCommandGroup {
    public StackCommand(RobotHardware robot) {
        super(
                new InstantCommand(() -> robot.armSystem.setPosition(50)),
                new InstantCommand(() -> robot.angleOfArm.customAngle(0)),
                new InstantCommand(() -> robot.claw.releaseBoth())
        );
        Globals.startIntake();
    }
}