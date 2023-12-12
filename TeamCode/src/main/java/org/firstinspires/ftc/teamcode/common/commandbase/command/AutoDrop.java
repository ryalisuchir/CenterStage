package org.firstinspires.ftc.teamcode.common.commandbase.command;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

//command to drop the pixel during auto
public class AutoDrop extends SequentialCommandGroup {
    public AutoDrop(Robot robot) {
        super(
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.angle.autoDrop()),
                        new InstantCommand(() -> robot.a.armAutoDrop())
                )
        );
    }
}