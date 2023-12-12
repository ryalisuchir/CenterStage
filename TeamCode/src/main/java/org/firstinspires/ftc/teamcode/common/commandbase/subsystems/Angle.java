package org.firstinspires.ftc.teamcode.common.commandbase.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Angle extends SubsystemBase {
    private final Servo dump;

    public static double intake_position = 0.65;
    public static double rest_position = 0.6;
    public static double outtake_position = 0.845;
    public static double autodrop_position = 0.845;

    public Angle(Servo d) {
        dump = d;
    }

    public void in() {
        dump.setPosition(intake_position);
    }

    public void rest() {
        dump.setPosition(rest_position);
    }

    public void out() {
        dump.setPosition(outtake_position);
    }
    public void autoDrop() {
        dump.setPosition(autodrop_position);
    }

}