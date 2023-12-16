package org.firstinspires.ftc.teamcode.common.commandbase.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class AngleSubsystem extends SubsystemBase {
    private final Servo dump;
    public static double intake_position = -0.1;
    public static double rest_position =0;
    public static double outtake_position = 0.43;
    public static double autodrop_position = 0.05;

    public AngleSubsystem(final HardwareMap hMap, final String name) {
        dump = hMap.get(Servo.class, "dump");
        dump.setDirection(Servo.Direction.REVERSE);
    }

    public void intake() {
        dump.setPosition(intake_position);
    }

    public void rest() {
        dump.setPosition(rest_position);
    }

    public void outtake() {
        dump.setPosition(outtake_position);
    }
    public void tapeDrop() {
        dump.setPosition(autodrop_position);
    }
}
