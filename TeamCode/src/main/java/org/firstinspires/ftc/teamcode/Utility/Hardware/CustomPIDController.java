package org.firstinspires.ftc.teamcode.Utility.Hardware;

public class CustomPIDController extends CustomPIDFController {

    public CustomPIDController(double kp, double ki, double kd) {
        super(kp, ki, kd, 0);
    }

    public CustomPIDController(double kp, double ki, double kd, double sp, double pv) {
        super(kp, ki, kd, 0, sp, pv);
    }

    public void setPID(double kp, double ki, double kd) {
        setPIDF(kp, ki, kd, 0);
    }

}