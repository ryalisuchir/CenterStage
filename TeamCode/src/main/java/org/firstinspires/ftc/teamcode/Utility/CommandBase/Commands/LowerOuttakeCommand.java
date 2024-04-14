package org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Utility.Hardware.Globals;
import org.firstinspires.ftc.teamcode.Utility.Hardware.RobotHardware;

public class LowerOuttakeCommand extends ParallelCommandGroup {
    public LowerOuttakeCommand(RobotHardware robot) {
        super(
                new InstantCommand(() -> robot.angleOfArm.outtake()),
                new InstantCommand(() -> robot.armSystem.armOuttake()),
                new WaitCommand(1000),
                new InstantCommand(() -> robot.slidesSubsystem.lowerOuttake())
        );
        Globals.startOuttake();
    }
}