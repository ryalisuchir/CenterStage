package org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Utility.Hardware.Globals;
import org.firstinspires.ftc.teamcode.Utility.Hardware.RobotHardware;

public class RestCommand extends SequentialCommandGroup {
    public RestCommand(RobotHardware robot) {
        super(
                new WaitCommand(750),
                new InstantCommand(() -> robot.slidesSubsystem.intake()),
                new WaitCommand(750),
                new InstantCommand(() -> robot.armSystem.armCoast()),
                new InstantCommand(() -> robot.angleOfArm.rest()),
                new InstantCommand(() -> robot.claw.grabBoth())
        );
        Globals.goToRest();
    }
}