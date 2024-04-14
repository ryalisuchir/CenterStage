package org.firstinspires.ftc.teamcode.Utility.CommandBase.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class ClawSubsystem extends SubsystemBase {
    private final Servo clawLeft, clawRight;
//    public static double grabPositionLeft = 0, grabPositionRight = 0.85;
//    public static double releasePositionLeft = 0.85, releasePositionRight = 0, smallReleasePositionLeft = 0.65, smallReleasePositionRight = 0.68;
public static double grabPositionLeft = 0.05, grabPositionRight = 0.7;
    public static double releasePositionLeft = 1, releasePositionRight = 0, smallReleasePositionLeft = 0.65, smallReleasePositionRight = 0.326;

    public ClawSubsystem(final HardwareMap hMap, final String leftClaw, final String rightClaw) {
        clawLeft = hMap.get(Servo.class, leftClaw);
        clawRight = hMap.get(Servo.class, rightClaw);
    }
    public void grabBoth() {
        clawLeft.setPosition(grabPositionLeft);
        clawRight.setPosition(grabPositionRight);
    }
    public void customLeft(double pos) {clawLeft.setPosition(pos);}

    public void customRight(double pos) {clawRight.setPosition(pos);}
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
    public void smallReleaseLeft() {
        clawLeft.setPosition(smallReleasePositionLeft);
    }
    public void smallReleaseRight() {
        clawRight.setPosition(smallReleasePositionRight);
    }

}