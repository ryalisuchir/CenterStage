package org.firstinspires.ftc.teamcode.common.commandbase.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Claw extends SubsystemBase {
    private final Servo clawLeft, clawRight;
    // * can be tuned in dashboard
    public static double grabPositionLeft = 0.2, grabPositionRight = 0.3; //MODIFY
    public static double releasePositionLeft = 0.5, releasePositionRight = 0.05; //MODIFY

    public Claw(Servo clawLeft, Servo clawRight) {
        this.clawLeft = clawLeft;
        this.clawRight = clawRight;
    }

    public Command grab() {
        return new InstantCommand(() -> {
            clawLeft.setPosition(grabPositionLeft);
            clawRight.setPosition(grabPositionRight);
        }, this).andThen(
                new WaitCommand(500)
        );
    }

    public Command releaseLeft() {
        return new InstantCommand(() -> {
            clawLeft.setPosition(releasePositionLeft);
        }, this).andThen(
                new WaitCommand(500)
        );
    }

    public Command releaseRight() {
        return new InstantCommand(() -> {
            clawRight.setPosition(releasePositionRight);
        }, this).andThen(
                new WaitCommand(500)
        );
    }

    public Command releaseBoth() {
        return new InstantCommand(() -> {
            clawLeft.setPosition(releasePositionLeft);
            clawRight.setPosition(releasePositionRight);
        }, this).andThen(
                new WaitCommand(500)
        );
    }
}