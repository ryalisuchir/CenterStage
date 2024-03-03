package org.firstinspires.ftc.teamcode.Utility.CommandBase.Subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class AngleSubsystem extends SubsystemBase {
    private final Servo dump;

    public AngleSubsystem(final HardwareMap hMap, final String servoName) {
        dump = hMap.get(Servo.class, servoName);
        dump.setDirection(Servo.Direction.REVERSE);
    }

    public void customAngle(double angle) {
        dump.setPosition(angle);
    }
    public void intake() {
        dump.setPosition(0.15);
    }
    public void stack() {
        dump.setPosition(0.19);
    }
    public void newStack() {
        dump.setPosition(0.198);
    }
    public void outtake() {
        dump.setPosition(0.57);
    }

    public void doubleOuttake() {dump.setPosition(0.555);}
    public void rest() {
        dump.setPosition(0.3);
    }
    public void tapeDrop() {
        dump.setPosition(0.1);
    }
}