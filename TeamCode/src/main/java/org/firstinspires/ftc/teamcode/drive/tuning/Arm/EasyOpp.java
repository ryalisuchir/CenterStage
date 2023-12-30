package org.firstinspires.ftc.teamcode.drive.tuning.Arm;
//opp? more like op. get it? cause opmode. GRATATATATA

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp
public class EasyOpp extends OpMode {
    private PIDController controller;
    public static double p = 0, i = 0, d = 0;
    public static double f = 0;
    public double previousPower;
    public static double MAX_ACCELERATION = 0.5;
    public static double anglePos = 0;
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
    }
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void loop() {
        dump.setPosition(anglePos);
        controller.setPID(p, i, d);
        int armPos = arm.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;

        double powerDerivative = (power - previousPower) / timer.seconds();
        if (powerDerivative > MAX_ACCELERATION) {
            power = previousPower + (MAX_ACCELERATION * timer.seconds());
        } else if (powerDerivative < -MAX_ACCELERATION) {
            power = previousPower - (MAX_ACCELERATION * timer.seconds());
        }

        arm.setPower(power);
        timer.reset();

        telemetry.addData("arm position:", armPos);
        telemetry.addData("target:", target);
        telemetry.update();
    }
}

