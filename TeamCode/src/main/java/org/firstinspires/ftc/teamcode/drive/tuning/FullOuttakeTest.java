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
public class FullOuttakeTest extends OpMode {
    private PIDController controller;
    public static double p = 0, i = 0, d = 0;
    public static double f = 0;
    public static double servoPos = 0;
    public static int target = 0;
    private final double ticks_in_degree = 384.5 / 180; //depends on motor you use
    private DcMotorEx linear_1, linear_2;
    Servo dump;

    //arm stuff:
    private PIDController controller2;
    public static double p2 = 0.01, i2 = 0, d2 = 0.00083;
    public static double f2 = 0.008;

    public static int target2 = 0;
    private final double ticks_in_degree2 = 384.5 / 180; //depends on motor you use
    private DcMotorEx arm;
    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        linear_1 = hardwareMap.get(DcMotorEx.class, "linear_1");
        linear_2 = hardwareMap.get(DcMotorEx.class, "linear_2");
        linear_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linear_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linear_2.setDirection(DcMotor.Direction.REVERSE);

        dump = hardwareMap.get(Servo.class, "dump");
        dump.setPosition(0);

        //arm stuff:
        controller2 = new PIDController(p2, i2, d2);
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        dump.setPosition(servoPos);
        controller.setPID(p, i, d);
        int linearPos = linear_1.getCurrentPosition();
        double pid = controller.calculate(linearPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;

        linear_1.setPower(power);
        linear_2.setPower(power);

        telemetry.addData("linear pos:", linearPos);
        telemetry.addData("linear target:", target);

        //arm stuff:
        controller2.setPID(p2, i2, d2);
        int armPos = arm.getCurrentPosition();
        double pid2 = controller.calculate(armPos, target2);
        double ff2 = Math.cos(Math.toRadians(target / ticks_in_degree2)) * f2;

        double power2 = pid2 + ff2;

        if (power2 > 0.6) {
            power2 = 0.6;
        }
        if (power2 < -0.6) {
            power2 = -0.6;
        }

        arm.setPower(power2);

        telemetry.addData("arm pos: ", armPos);
        telemetry.addData("arm target:", target2);

        telemetry.update();
    }
}
