package org.firstinspires.ftc.teamcode.Drive.Tuning;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
@Disabled
@Photon
public class SlidesTesting extends OpMode {
    private PIDController controller;
    private DcMotorEx linear_1, linear_2;
    public static int target = 800;

    @Override
    public void init() {
        linear_1 = hardwareMap.get(DcMotorEx.class, "linear_1");
        linear_2 = hardwareMap.get(DcMotorEx.class, "linear_2");

        linear_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear_1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linear_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        linear_2.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void loop() {

        if (linear_2.getCurrentPosition() < -800) {
            linear_1.setPower(0.1);
            linear_2.setPower(0.1);
        } else if (linear_2.getCurrentPosition() > -800) {
            linear_1.setPower(0.6);
            linear_2.setPower(0.6);
        }

        telemetry.addData("Linear 1 Position: ", linear_1.getCurrentPosition());
        telemetry.addData("Linear 2 Position: ", linear_2.getCurrentPosition());
        telemetry.addData("Linear 1 Power: ", linear_1.getPower());
        telemetry.addData("Linear 2 Power: ", linear_2.getPower());
        telemetry.update();
    }
}