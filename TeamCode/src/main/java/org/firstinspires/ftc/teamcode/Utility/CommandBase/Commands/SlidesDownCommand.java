package org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Utility.Hardware.Globals;
import org.firstinspires.ftc.teamcode.Utility.Hardware.RobotHardware;

public class SlidesDownCommand extends SequentialCommandGroup {
    public SlidesDownCommand(RobotHardware robot) {
        super(
                new InstantCommand(() -> robot.slidesSubsystem.intake()),
                new InstantCommand(() -> robot.claw.grabBoth())
        );
        Globals.goToRest();
    }
}