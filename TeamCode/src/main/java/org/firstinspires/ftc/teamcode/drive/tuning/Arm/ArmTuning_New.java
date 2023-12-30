package org.firstinspires.ftc.teamcode.drive.tuning.Arm;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class ArmTuning_New extends OpMode {
    private PIDController controller;
    public double p = 0, i = 0, d = 0;
    public double f = 0.001;
    private double EncoderCPR = 1425.1;
    private final double GEAR_RATIO = 1.0;
    private double ARM_TICKS_PER_DEGREE = EncoderCPR * GEAR_RATIO / 360.0;
    private final double ZERO_OFFSET = 23.0; //manipulate based on where 0 value is

    public DcMotorEx armMotor;
    private double targetPosInDegrees;
    private double powerLimit;

    public double armPosition = 180;
    public static double armPower = 1;
    public double dumpPosition;
    public double ff, power;
    Servo dump;

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
    }

    @Override
    public void loop() {
        dump.setPosition(dumpPosition);
        setPosition(armPosition, armPower);
        armTask();
        telemetry.addData("arm position:", armMotor.getCurrentPosition());
        telemetry.addData("target:", armPosition);
        telemetry.addData("degrees: ", ticksToRealWorldDegrees(armMotor.getCurrentPosition()));
        telemetry.addData("ff: ", ff);
        telemetry.addData("power: ", armPower);
        telemetry.addData("angle of sin: ", Math.sin(Math.toRadians(ticksToRealWorldDegrees(armMotor.getCurrentPosition()))));
        telemetry.addData("f: ", f);
        telemetry.addData("POWER: ", power);
        telemetry.update();
    }
    public void armTask()
    {

        double targetPosInTicks = (targetPosInDegrees - ZERO_OFFSET) * ARM_TICKS_PER_DEGREE;
        double currPosInTicks = armMotor.getCurrentPosition();
        double pidOutput = controller.calculate(currPosInTicks, targetPosInTicks);
        // This ff is assuming arm at horizontal position is 90-degree.
        ff = (double) f * Math.sin(Math.toRadians(ticksToRealWorldDegrees(currPosInTicks)));
        power = pidOutput + ff;
        // Clip power to the range of -powerLimit to powerLimit.
        power = power < -powerLimit ? -powerLimit : power > powerLimit ? powerLimit : power;
        armMotor.setPower(power);
        //armMotor.setPower(0.1);
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

