package org.firstinspires.ftc.teamcode.Drive.Tuning;

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

@Config
@TeleOp
public class SlidesTuning extends OpMode {
    private PIDController controller;
    public static double p = 0, i = 0, d = 0;
    public static double f = 0;
    public static int target = 0;
    private final double ticks_in_degree = 751.8 / 180; //depends on motor you use
    private DcMotorEx linear_1, linear_2;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        linear_1 = hardwareMap.get(DcMotorEx.class, "linear_1");
        linear_2 = hardwareMap.get(DcMotorEx.class, "linear_2");
        linear_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linear_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linear_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        linear_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linear_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        linear_2.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int linearPos = -linear_1.getCurrentPosition();
        int linearPos2 = -linear_2.getCurrentPosition();

        double pid = controller.calculate(linearPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;
        //        if (power > 0.08) {
        //            power = 0.08;
        //        };
        //
        //        if (power < -0.08) {
        //            power = -0.08;
        //        };

        linear_1.setPower(power);
        linear_2.setPower(power);

        telemetry.addData("Linear 1:", linearPos);
        telemetry.addData("Linear 2:", linearPos2);
        telemetry.addData("target:", target);
        telemetry.update();
    }
}