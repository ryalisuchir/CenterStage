package org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Utility.Hardware.Globals;
import org.firstinspires.ftc.teamcode.Utility.Hardware.RobotHardware;

public class RestCommand extends SequentialCommandGroup {
    public RestCommand(RobotHardware robot) {
        super(
                new InstantCommand(() -> robot.slidesSubsystem.intake()),
                new InstantCommand(() -> robot.armSystem.armCoast()),
                new InstantCommand(() -> robot.angleOfArm.rest())
        );
        Globals.goToRest();
    }
}