package org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import org.firstinspires.ftc.teamcode.Utility.Hardware.RobotHardware;

public class TwoPixelDropCommand extends SequentialCommandGroup {
    public TwoPixelDropCommand(RobotHardware robot) {
        super(
                new InstantCommand(() -> robot.claw.smallReleaseLeft()),
                new WaitCommand(1000),
                new InstantCommand(() -> robot.claw.releaseLeft())
        );
    }
}