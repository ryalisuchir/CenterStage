package org.firstinspires.ftc.teamcode.Utility.CommandBase.Subsystems;

import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Utility.Hardware.CustomPIDController;
import org.firstinspires.ftc.teamcode.Utility.Hardware.Globals;


import java.util.function.DoubleSupplier;

@Config
public class SlowerArmSubsystem extends SubsystemBase {
    public final DcMotorEx arm;

    private final VoltageSensor batteryVoltageSensor;

    public double p = 0;
    public double i = 0;
    public double d = 0;
    public double f = 0;

    private final double ticks_to_degrees = 1425.1 / 360;
    private final double zeroOffset = 23.0;

    private final CustomPIDController controller;
    private ElapsedTime time;
    private ElapsedTime voltageTimer;
    private double voltage;

    private MotionProfile profile;
    public static double max_v = 10000;
    public static double max_a = 6000;

    private int target = 0;
    private int previous_target = 5;

    private double cache = 0;

    public SlowerArmSubsystem(DcMotorEx a, VoltageSensor b) {
        arm = a;
        controller = new CustomPIDController(p, i, d);
        controller.setPID(p, i, d);
        this.batteryVoltageSensor = b;
        time = new ElapsedTime();
        voltageTimer = new ElapsedTime();
        voltage = batteryVoltageSensor.getVoltage();

    }

    public void loop() {

        if(arm.getCurrentPosition() >= 70) {
            arm.setPower(0);
        } else if(arm.getCurrentPosition() >= 40 && arm.getCurrentPosition() < 70) {
            arm.setPower(0.05);
        } else if(arm.getCurrentPosition() >= 20 && arm.getCurrentPosition() < 40) {
            arm.setPower(0.2);
        } else if(arm.getCurrentPosition() < 20) {
            arm.setPower(0.34);
        }

    }

    public void setPosition(int position) {
        target = position;
    }
    public void armOuttake() {
        target = 650;
    }
    public void armCoast() {
        target = 20;
    }

}