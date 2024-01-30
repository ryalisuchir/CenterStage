package org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.SlowerCommands;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Utility.Hardware.Globals;
import org.firstinspires.ftc.teamcode.Utility.Hardware.RobotHardware;

public class SlowRestCommand extends SequentialCommandGroup {
    public SlowRestCommand(RobotHardware robot) {
        super(
                new InstantCommand(() -> robot.slidesSubsystem.intake()),
                new InstantCommand(() -> robot.slowArmSystem.armCoast()),
                new InstantCommand(() -> robot.angleOfArm.rest())
        );
        Globals.goToRest();
    }
}