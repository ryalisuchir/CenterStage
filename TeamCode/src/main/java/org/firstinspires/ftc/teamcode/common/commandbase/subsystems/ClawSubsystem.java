package org.firstinspires.ftc.teamcode.common.commandbase.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawSubsystem extends SubsystemBase {
    private final Servo clawLeft, clawRight;
    public static double grabPositionLeft = 0, grabPositionRight = 1; //MODIFY
    public static double releasePositionLeft = 1, releasePositionRight = 0; //MODIFY

    public ClawSubsystem(final HardwareMap hMap, final String name, final String name2) {
        clawLeft = hMap.get(Servo.class, name);
        clawRight = hMap.get(Servo.class, name2);
    }

    public void grabBoth() {
        clawLeft.setPosition(grabPositionLeft);
        clawRight.setPosition(grabPositionRight);
    }

    public void grabLeft() {
        clawLeft.setPosition(grabPositionLeft);
    }

    public void grabRight() {
        clawRight.setPosition(grabPositionRight);
    }

    public void releaseLeft() {
        clawLeft.setPosition(releasePositionLeft);
    }

    public void releaseRight() {
        clawRight.setPosition(releasePositionRight);
    }

    public void releaseBoth() {
        clawLeft.setPosition(releasePositionLeft);
        clawRight.setPosition(releasePositionRight);

    }
}
