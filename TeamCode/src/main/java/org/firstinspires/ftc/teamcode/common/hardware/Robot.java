package org.firstinspires.ftc.teamcode.common.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.AngleSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.Slides;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.Drive;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
public class Robot {
    public DcMotorEx leftFront, rightFront, leftRear, rightRear, linear_1, linear_2, arm;
    public Servo dump, claw1, claw2;
    public VoltageSensor batteryVoltageSensor;

    public AngleSubsystem angle;

    public ArmSubsystem a;
    public ClawSubsystem claw;
    public Slides slides;

    public Drive drive;
    public static double MAX_CURRENT = 15;

    public Robot(HardwareMap hardwareMap) {
        //set drivetrain
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        //brake
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //reverse
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        //set external motors
        linear_1 = hardwareMap.get(DcMotorEx.class, "linear_1");
        linear_2 = hardwareMap.get(DcMotorEx.class, "linear_2");
        arm = hardwareMap.get(DcMotorEx.class, "arm");

        linear_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linear_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        linear_2.setDirection(DcMotor.Direction.REVERSE);
        linear_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linear_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //servos
        Servo dump = hardwareMap.get(Servo.class, "dump");
        Servo claw1 = hardwareMap.get(Servo.class, "claw");
        Servo claw2 = hardwareMap.get(Servo.class, "claw1");


        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        //setting subsystems
        a = new ArmSubsystem(arm, batteryVoltageSensor);
        claw = new ClawSubsystem(hardwareMap, "claw", "claw1");
        angle = new AngleSubsystem(hardwareMap, "dump");
        drive = new Drive(new SampleMecanumDrive(hardwareMap), false);

        CommandScheduler.getInstance().registerSubsystem(a, claw, angle, drive);

    }

}