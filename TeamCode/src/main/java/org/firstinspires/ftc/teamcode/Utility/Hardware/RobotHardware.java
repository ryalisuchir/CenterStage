package org.firstinspires.ftc.teamcode.Utility.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Subsystems.AngleSubsystem;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Subsystems.SlidesSubsystem;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Subsystems.SlowerArmSubsystem;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@Config
public class RobotHardware {
    public DcMotorEx leftFront, rightFront, leftRear, rightRear, linear_1, linear_2, arm;
    public Servo angleOfClaw, leftClaw, rightClaw;
    public VoltageSensor batteryVoltageSensor;

    public AngleSubsystem angleOfArm;
    public ArmSubsystem armSystem;
    public SlowerArmSubsystem slowArmSystem;
    public ClawSubsystem claw;
    public DriveSubsystem driveSubsystem;
    public SlidesSubsystem slidesSubsystem;
    public MecanumDrive drive;

    public RobotHardware(HardwareMap hardwareMap) {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        linear_1 = hardwareMap.get(DcMotorEx.class, "linear_1");
        linear_2 = hardwareMap.get(DcMotorEx.class, "linear_2");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setDirection(DcMotor.Direction.REVERSE);

        linear_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linear_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        linear_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear_1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linear_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        linear_2.setDirection(DcMotor.Direction.REVERSE);


        Servo angleOfClaw = hardwareMap.get(Servo.class, "dump");
        Servo leftClaw = hardwareMap.get(Servo.class, "claw");
        Servo rightClaw = hardwareMap.get(Servo.class, "claw1");


        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        armSystem = new ArmSubsystem(arm, batteryVoltageSensor);
        slowArmSystem = new SlowerArmSubsystem(arm, batteryVoltageSensor);
        claw = new ClawSubsystem(hardwareMap, "claw", "claw1");
        angleOfArm = new AngleSubsystem(hardwareMap, "dump");
        driveSubsystem = new DriveSubsystem(new SampleMecanumDrive(hardwareMap), false);
        slidesSubsystem = new SlidesSubsystem(linear_1, linear_2, batteryVoltageSensor);
        CommandScheduler.getInstance().registerSubsystem(armSystem, claw, angleOfArm, driveSubsystem, slidesSubsystem);

    }

}