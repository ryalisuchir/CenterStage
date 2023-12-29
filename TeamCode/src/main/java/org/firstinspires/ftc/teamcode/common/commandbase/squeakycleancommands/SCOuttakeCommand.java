package org.firstinspires.ftc.teamcode.common.commandbase.squeakycleancommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.hardware.Robot;

    public class SCOuttakeCommand extends SequentialCommandGroup {
        public SCOuttakeCommand(Robot robot) {
            super(
                    new ParallelCommandGroup(
                            new InstantCommand(() -> robot.sca.setPosition(60, 0.5)),
                            new InstantCommand(() -> robot.angle.outtake())
                    )
            );
        }
    }