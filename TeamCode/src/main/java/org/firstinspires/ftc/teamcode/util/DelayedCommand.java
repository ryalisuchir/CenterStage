package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

public class DelayedCommand extends SequentialCommandGroup {
    public DelayedCommand(Command command, int delay) {
        addCommands(
                new WaitCommand(delay),
                command
        );
    }
}