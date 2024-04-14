package org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Utility.Hardware.Globals;
import org.firstinspires.ftc.teamcode.Utility.Hardware.RobotHardware;

public class xPerimentalGrab extends SequentialCommandGroup {
    public xPerimentalGrab(RobotHardware robot) {
        super(
                new InstantCommand(() -> robot.angleOfArm.lolStack()),
                new WaitCommand(250),
                new InstantCommand(() -> robot.claw.grabRight()),
                new WaitCommand(250),
                new InstantCommand(() -> robot.angleOfArm.lowerStack())
        );

        Globals.startIntake();
    }
}