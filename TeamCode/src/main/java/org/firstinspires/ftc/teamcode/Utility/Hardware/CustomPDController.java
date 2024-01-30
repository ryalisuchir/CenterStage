package org.firstinspires.ftc.teamcode.Utility.Hardware;

public class CustomPDController extends CustomPIDController {

    /**
     * Default constructor with just the coefficients
     */
    public CustomPDController(double kp, double kd) {
        super(kp, 0, kd);
    }

    /**
     * The extended constructor.
     */
    public CustomPDController(double kp, double kd, double sp, double pv) {
        super(kp, 0, kd, sp, pv);
    }
}