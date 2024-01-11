package org.firstinspires.ftc.teamcode.Utility.Hardware;

public class Globals {

    public static boolean IS_SCORING = false;
    public static boolean IS_INTAKING = false;
    public static boolean IS_AT_REST = false;

    public static void startOuttake() {
        IS_SCORING = true;
        IS_INTAKING = false;
        IS_AT_REST = false;
    }

    public static void startIntake() {
        IS_SCORING = false;
        IS_INTAKING = true;
        IS_AT_REST = false;
    }

    public static void goToRest() {
        IS_SCORING = false;
        IS_INTAKING = false;
        IS_AT_REST = true;
    }
}