package org.firstinspires.ftc.teamcode.common.commandbase.squeakycleancommands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.DoubleSupplier;

@Config
public class SqueakyCleanBrandNewArmSubsystem extends SubsystemBase {
    public final DcMotorEx arm;
    private PIDController controller;
    public static double p = 0, i = 0, d = 0;
    private static final double YappYappaYoGabbaGaba = 1425.1; //encoder count but ankita is dumb
    private static final double GEAR_RATIO = 1.0;
    private static final double ARM_TICKS_PER_DEGREE = YappYappaYoGabbaGaba * GEAR_RATIO / 360.0;
    private static final double MAX_ARM_HOLDING_POWER = 0; //f component
    private static final double ZERO_OFFSET = 45.0; //manipulate based on where 0 value is

    public DcMotorEx armMotor;
    public Servo dump;
    private double targetPosInDegrees;
    private double powerLimit;

    InterpLUT pCoefficients = new InterpLUT();
    InterpLUT iCoefficients = new InterpLUT();
    InterpLUT dCoefficients = new InterpLUT();



    public SqueakyCleanBrandNewArmSubsystem(final HardwareMap hMap, DcMotorEx a, final String dumper) {
        arm = a;
        dump = hMap.get(Servo.class, dumper);

        controller = new PIDController(p, i, d);
        controller.setPID(p, i, d);

        pCoefficients.add(0, 0); //kp at 0 degrees
        pCoefficients.add(90, 0); //kp at 90 degrees

        iCoefficients.add(0, 0);
        iCoefficients.add(90, 0);

        dCoefficients.add(0, 0); //kd at 0 degrees
        dCoefficients.add(90, 0); //kd at 90 degrees

        pCoefficients.createLUT();
        iCoefficients.createLUT();
        dCoefficients.createLUT();
    }


    public void armLooper()
    {
        double armAngle = dump.getPosition();
        double Kp = pCoefficients.get(armAngle);
        double Ki = iCoefficients.get(armAngle);
        double Kd = dCoefficients.get(armAngle);

        controller.setPID(Kp, Ki, Kd);

        double targetPosInTicks = (targetPosInDegrees - ZERO_OFFSET) * ARM_TICKS_PER_DEGREE;
        double currPosInTicks = armMotor.getCurrentPosition();
        double pidOutput = controller.calculate(currPosInTicks, targetPosInTicks);
        // This ff is assuming arm at horizontal position is 90-degree.
        double ff = MAX_ARM_HOLDING_POWER * Math.sin(Math.toRadians(ticksToRealWorldDegrees(currPosInTicks)));
        double power = pidOutput + ff;
        // Clip power to the range of -powerLimit to powerLimit.
        power = power < -powerLimit ? -powerLimit : power > powerLimit ? powerLimit : power;
        armMotor.setPower(power);
    }

    public boolean isOnTarget(double toleranceInDegrees)
    {
        double currPosInDegrees = getPosition();
        return Math.abs(targetPosInDegrees - currPosInDegrees) <= toleranceInDegrees;
    }

    public void setPosition(double targetPosInDegrees, double powerLimit)
    {
        this.targetPosInDegrees = targetPosInDegrees;
        this.powerLimit = Math.abs(powerLimit);
    }

    public double ticksToRealWorldDegrees(double ticks)
    {
        return ticks / ARM_TICKS_PER_DEGREE + ZERO_OFFSET;
    }

    public double getPosition()
    {
        return ticksToRealWorldDegrees(armMotor.getCurrentPosition());
    }

}