package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.util.CustomPIDFProcessor;

public class CustomPIDController extends CustomPIDFProcessor {

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