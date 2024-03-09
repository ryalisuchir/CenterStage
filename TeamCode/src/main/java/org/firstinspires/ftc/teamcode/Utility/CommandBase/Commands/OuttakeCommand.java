package org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Utility.Hardware.Globals;
import org.firstinspires.ftc.teamcode.Utility.Hardware.RobotHardware;

public class OuttakeCommand extends ParallelCommandGroup {
    public OuttakeCommand(RobotHardware robot) {
        super(
                new InstantCommand(() -> robot.angleOfArm.outtake()),
                new InstantCommand(() -> robot.armSystem.armOuttake()),
                new InstantCommand(() -> robot.slidesSubsystem.outtake())
        );
        Globals.startOuttake();
    }
}