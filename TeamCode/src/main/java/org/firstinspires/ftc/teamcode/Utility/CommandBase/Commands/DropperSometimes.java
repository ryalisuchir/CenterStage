package org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.Utility.Hardware.RobotHardware;

public class DropperSometimes extends SequentialCommandGroup {
    public DropperSometimes(RobotHardware robot) {
        super(
                new InstantCommand(() -> robot.slidesSubsystem.intake()),
                new WaitCommand(2000),
                new InstantCommand(() -> robot.armSystem.armTapeDrop()),
                new InstantCommand(() -> robot.claw.grabBoth()),
                new InstantCommand(() -> robot.angleOfArm.tapeDrop())
        );
    }
}