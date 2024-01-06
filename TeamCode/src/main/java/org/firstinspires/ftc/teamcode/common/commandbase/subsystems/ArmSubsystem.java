package org.firstinspires.ftc.teamcode.common.commandbase.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.CustomPIDController;

import java.util.function.DoubleSupplier;

@Config
public class ArmSubsystem extends SubsystemBase {
    public final DcMotorEx arm;

    private final VoltageSensor batteryVoltageSensor;


    private final double p = 0.01; //adjust TODO
    private final double d = 0; //adjust TODO
    private final double f = 0; //adjust TODO //0.009
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

    public ArmSubsystem(DcMotorEx a, VoltageSensor b) {
        arm = a;

        controller = new CustomPIDController(p, 0, d);
        controller.setPID(p, 0, d);
        this.batteryVoltageSensor = b;
        time = new ElapsedTime();
        voltageTimer = new ElapsedTime();
        voltage = batteryVoltageSensor.getVoltage();

    }

    public void loop() {
        if (target != previous_target) {
            profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(previous_target, 0), new MotionState(target, 0), max_v, max_a);
            time.reset();
            previous_target = target;
        }

        if (voltageTimer.seconds() > 5) {
            voltage = batteryVoltageSensor.getVoltage();
            voltageTimer.reset();
        }

        int armpos = arm.getCurrentPosition();
        cache = armpos;
        MotionState targetState = profile == null ? new MotionState(0, 0) : profile.get(time.seconds());
        double target = targetState.getX();
        double pid = controller.calculate(armpos, target);
        double ff = Math.sin(Math.toRadians(target / ticks_to_degrees + zeroOffset )) * f;

        double power = (pid + ff) / voltage * 12.0;

        if (power > 0.2) {
            power = 0.2;
        }
        if (power < -0.2) {
            power = -0.2;
        }

        arm.setPower(power);

    }

    public void setPos(int pos) {
        target = pos;
    }

    public void armIntake() {
        target = 0; //adjust TODO
    }

    public void armOuttake() {
        target = 650; //adjust TODO
    }

    public void armCoast() {
        target = 30; //adjust TODO
    }
    public void armTapeDrop() {
        target = 10; //adjust TODO
    }


    public int pos() {
        return arm.getCurrentPosition();
    }

    public void adjustArm(DoubleSupplier percentage) {
        target = 705 + (int) (50 * percentage.getAsDouble());
    }

    public double getCachePos() {
        return cache;
    }
}