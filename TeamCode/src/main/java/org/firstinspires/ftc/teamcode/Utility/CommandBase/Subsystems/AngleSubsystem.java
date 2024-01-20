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
        dump.setPosition(0);
    }
    public void outtake() {
        dump.setPosition(0.56);
    }
    public void rest() {
        dump.setPosition(0.2);
    }
    public void tapeDrop() {
        dump.setPosition(0.05);
    }
}