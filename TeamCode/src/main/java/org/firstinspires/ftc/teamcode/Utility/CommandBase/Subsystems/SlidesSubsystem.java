package org.firstinspires.ftc.teamcode.Utility.CommandBase.Subsystems;

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

    private final double p = 0.002;
    private final double d = 0;
    private final double f = 0.00001;
    private final double ticks_to_degrees = 751.8 / 180.0;

    private final PIDController controller;
    private int target = 5;
    private double cache = 0;

    public SlidesSubsystem(DcMotorEx leftLinearMotor, DcMotorEx rightLinearMotor, VoltageSensor c) {
        linear_1 = leftLinearMotor;
        linear_2 = rightLinearMotor;

        linear_1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        linear_1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        controller = new PIDController(p, 0, d);
        controller.setPID(p, 0, d);
    }

    public void loop() {
        controller.setPID(p, 0, d);
        int armPos = linear_1.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_to_degrees)) * f;

        double power = pid + ff;

        if (power > 0.1) {
            power = 0.1;
        };

        if (power < -0.1) {
            power = -0.1;
        };

        linear_1.setPower(power);
        linear_2.setPower(power);

    }

    public void setPos(int pos) {
        target = pos;
    }

    public void outtake() {
        target = 866;
    }
    public void intake() {
        target = 0;
    }

    public double getCachePos() {
        return cache;
    }
}