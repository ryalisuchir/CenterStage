package org.firstinspires.ftc.teamcode.common.commandbase.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Angle extends SubsystemBase {
    private final Servo dump;
    //servo.setPosition(Range.clip(0, 1, servo.getPosition() + .01));
    //experiment with that to test if speed of servo can be changed ^^

    public static double intake_position = 0.65;
    public static double rest_position = 0.6;
    public static double outtake_position = 0.845;
    public static double autodrop_position = 0.845;

    public Angle(Servo d) {
        dump = d;
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