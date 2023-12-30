package org.firstinspires.ftc.teamcode.drive.tuning;


import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import java.time.Duration;
import java.time.Instant;

@Config
@TeleOp
public class ArmTuning extends OpMode {
    private final double zeroOffset = 23.0;
    private PIDController controller;
    public static double p = 0.01, i = 0, d = 0.00083;
    public static double f = 0.009;
    public static double servoPos = 0;
    public static int target = 0;
    private final double ticks_in_degree = 1425.1 / 360; //depends on motor you use
    private DcMotorEx arm;
    Servo dump;
    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setDirection(DcMotor.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dump = hardwareMap.get(Servo.class, "dump");
        dump.setDirection(Servo.Direction.REVERSE);
        dump.setPosition(0);
    }

    @Override
    public void loop() {
        dump.setPosition(servoPos);
        controller.setPID(p, i, d);
        int armPos = arm.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.sin(Math.toRadians(armPos / ticks_in_degree + zeroOffset )) * f;

        double power = pid + ff;

        arm.setPower(power);

        telemetry.addData("pos1:", armPos);
        telemetry.addData("target:", target);
        telemetry.update();
    }

}