package org.firstinspires.ftc.teamcode.drive.tuning.Arm;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class GrahGrahOpp extends OpMode {
    private PIDController controller;
    public static double p = 0, i = 0, d = 0;
    private static final double YappYappaYoGabbaGaba = 1425.1; //encoder count but ankita is dumb
    private static final double GEAR_RATIO = 1.0;
    private static final double ARM_TICKS_PER_DEGREE = YappYappaYoGabbaGaba * GEAR_RATIO / 360.0;
    private static final double MAX_ARM_HOLDING_POWER = 0; //f component
    private static final double ZERO_OFFSET = 45.0; //manipulate based on where 0 value is

    public DcMotorEx armMotor;
    private double targetPosInDegrees;
    private double powerLimit;

    public static double armPosition = 0;
    public static double armPower = 0;
    public static double dumpPosition;
    Servo dump;

    InterpLUT pCoefficients = new InterpLUT();
    InterpLUT iCoefficients = new InterpLUT();
    InterpLUT dCoefficients = new InterpLUT();


    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        armMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dump = hardwareMap.get(Servo.class, "dump");
        dump.setDirection(Servo.Direction.REVERSE);

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

    @Override
    public void loop() {
        double armAngle = dump.getPosition();

        double Kp = pCoefficients.get(armAngle);
        double Ki = iCoefficients.get(armAngle);
        double Kd = dCoefficients.get(armAngle);

        controller.setPID(Kp, Ki, Kd);

        dump.setPosition(dumpPosition);
        setPosition(armPosition, armPower);
        armTask();
    }
    public void armTask()
    {
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

