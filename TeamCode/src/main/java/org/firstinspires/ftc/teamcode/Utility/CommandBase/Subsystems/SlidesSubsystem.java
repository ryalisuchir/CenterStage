package org.firstinspires.ftc.teamcode.Utility.CommandBase.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Config
public class SlidesSubsystem extends SubsystemBase {
    public final DcMotorEx linear_1, linear_2;
    private int target = 5;
    private double cache = 0;
    private boolean up = false;

    public SlidesSubsystem(DcMotorEx leftLinearMotor, DcMotorEx rightLinearMotor, VoltageSensor c) {
        linear_1 = leftLinearMotor;
        linear_2 = rightLinearMotor;

        linear_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear_1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linear_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        linear_2.setDirection(DcMotor.Direction.REVERSE);
    }

    public void loop() {

        if (up) {
            if (linear_2.getCurrentPosition() < -target) {
                linear_1.setPower(0.1);
                linear_2.setPower(0.1);
            } else if (linear_2.getCurrentPosition() > -target) {
                linear_1.setPower(1);
                linear_2.setPower(1);
            }
        } else {
            if (linear_2.getCurrentPosition() < -target) {
                linear_1.setPower(-0.3);
                linear_2.setPower(-0.3);
            } else if (linear_2.getCurrentPosition() >= -target) {
                linear_1.setPower(0);
                linear_2.setPower(0);
            }
        }

    }

    public void setPos(int pos) {
        target = pos;
    }

    public void lowerOuttake() {
        target = 120;
        up = true;
    }
    public void outtake() {
        target = 180;
        up = true;
    }
    public void highOuttake() {
        target = 620;
        up = true;
    }

    public void superDuperHighOuttakeCommand() {
        target = 1800;
        up = true;
    }

    public void mediumOuttake() {
        target = 500;
        up = true;
    }

    public void superHighOuttakeCommand() {
        target = 1200;
        up = true;
    }
    public void intake() {
        target = 0;
        up = false;
    }

    public double getCachePos() {
        return cache;
    }
}