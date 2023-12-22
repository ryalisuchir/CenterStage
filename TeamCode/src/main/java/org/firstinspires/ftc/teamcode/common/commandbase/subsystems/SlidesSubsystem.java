package org.firstinspires.ftc.teamcode.common.commandbase.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.DoubleSupplier;

@Config
public class SlidesSubsystem extends SubsystemBase {
    public final DcMotorEx linear_1, linear_2;

    private final VoltageSensor batteryVoltageSensor;

    private final double p = 0.002; //adjust TODO
    private final double d = 0; //adjust TODO
    private final double f = 0.00001; //adjust TODO
    private final double ticks_to_degrees = 384.5 / 180.0;

    private final PIDController controller;
    private ElapsedTime time;
    private ElapsedTime voltageTimer;
    private double voltage;

    private MotionProfile profile;
    public static double max_v = 10000;
    public static double max_a = 6000;

    private int target = 5;
    private int previous_target = 5;


    private double cache = 0;

    public SlidesSubsystem(DcMotorEx a, DcMotorEx b, VoltageSensor c) {
        linear_1 = a;
        linear_2 = b;
        linear_2.setDirection(DcMotorEx.Direction.REVERSE);
        linear_1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        linear_1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        controller = new PIDController(p, 0, d);
        controller.setPID(p, 0, d);

        this.batteryVoltageSensor = c;
        time = new ElapsedTime();
        voltageTimer = new ElapsedTime();
        voltage = batteryVoltageSensor.getVoltage();
    }

    public void loop() {
        controller.setPID(p, 0, d);
        int armPos = linear_1.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_to_degrees)) * f;

        double power = pid + ff;

        linear_1.setPower(power);
        linear_2.setPower(power);

    }

    public void setPos(int pos) {
        target = pos;
    }

    public void toGround() {
        target = 5; //adjust TODO
    }

    public void autoOuttake() {
        target = -400; //adjust TODO
    }

    public void autoIntake() {
        target = 0; //adjust TODO
    }


    public int pos() {
        return linear_1.getCurrentPosition();
    }

    public void adjustArm(DoubleSupplier percentage) {
        target = 705 + (int) (50 * percentage.getAsDouble());
    }

    public double getCachePos() {
        return cache;
    }
}