package org.firstinspires.ftc.teamcode.common.commandbase.command;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

//description: ready to drop
public class OuttakeCommand extends SequentialCommandGroup {
    public OuttakeCommand(Robot robot) {
        super(
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.a.armOuttake()),
                        new InstantCommand(() -> robot.angle.rest())
                ),
                new WaitCommand(200),
                new InstantCommand(() -> robot.angle.outtake())
        );
    }
}