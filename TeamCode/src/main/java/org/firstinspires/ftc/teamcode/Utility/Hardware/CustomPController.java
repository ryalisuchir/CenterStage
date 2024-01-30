package org.firstinspires.ftc.teamcode.Utility.Hardware;

import org.firstinspires.ftc.teamcode.Utility.Hardware.CustomPDController;

public class CustomPController extends CustomPDController {

    /**
     * Default constructor, only takes a p-value.
     *
     * @param kp The value of kP for the coefficients.
     */
    public CustomPController(double kp) {
        super(kp, 0);
    }

    /**
     * The extended constructor.
     */
    public CustomPController(double kp, double sp, double pv) {
        super(kp, 0, sp, pv);
    }

}