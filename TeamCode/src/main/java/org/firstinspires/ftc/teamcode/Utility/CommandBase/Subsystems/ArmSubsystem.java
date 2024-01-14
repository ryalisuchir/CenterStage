package org.firstinspires.ftc.teamcode.Utility.CommandBase.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utility.Hardware.CustomPIDController;
import org.firstinspires.ftc.teamcode.Utility.Hardware.Globals;

import java.util.function.DoubleSupplier;

@Config
public class ArmSubsystem extends SubsystemBase {
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

    public ArmSubsystem(DcMotorEx a, VoltageSensor b) {
        arm = a;

        controller = new CustomPIDController(p, i, d);
        controller.setPID(p, i, d);
        this.batteryVoltageSensor = b;
        time = new ElapsedTime();
        voltageTimer = new ElapsedTime();
        voltage = batteryVoltageSensor.getVoltage();

    }

    public void loop() {

        if (Globals.IS_AT_REST) {
            p = 0.01;
            i = 0;
            d = 0.0001;
            f = 0.009;
        };

        if (Globals.IS_SCORING) {
            p = 0.01;
            i = 0;
            d = 0.0001;
            f = 0.009;
        };

        if (Globals.IS_INTAKING) {
            p = 0.01;
            i = 0;
            d = 0.0001;
            f = 0.009;
        };

        controller.setPID(p, i, d);

        if (target != previous_target) {
            profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(previous_target, 0), new MotionState(target, 0), max_v, max_a);
            time.reset();
            previous_target = target;
        }

        if (voltageTimer.seconds() > 5) {
            voltage = batteryVoltageSensor.getVoltage();
            voltageTimer.reset();
        }

        int currentArmPosition = arm.getCurrentPosition();
        cache = currentArmPosition;
        MotionState targetState = profile == null ? new MotionState(0, 0) : profile.get(time.seconds());
        double target = targetState.getX();
        double pid = controller.calculate(currentArmPosition, target);
        double ff = Math.sin(Math.toRadians(target / ticks_to_degrees + zeroOffset)) * f;

        double power = (pid + ff) / voltage * 12.0;

        if (power > 0.2) {
            power = 0.2;
        }
        if (power < -0.2) {
            power = -0.2;
        }

        arm.setPower(power);

    }

    public void setPosition(int position) {
        target = position;
    }
    public void armIntake() {
        target = 0;
    }
    public void armOuttake() {
        target = 650;
    }
    public void armCoast() {
        target = 30;
    }
    public void stackGrab() {
        target = 45;
    } //TODO: edit this!
    public void armTapeDrop() {
        target = 10;
    }

    public double getCachePos() {
        return cache;
    }
}