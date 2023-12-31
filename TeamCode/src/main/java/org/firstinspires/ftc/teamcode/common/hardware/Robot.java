package org.firstinspires.ftc.teamcode.common.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.AngleSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.SlidesSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.DriveSubsystem;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@Config
public class Robot {
    public DcMotorEx leftFront, rightFront, leftRear, rightRear, linear_1, linear_2, arm;
    public Servo dump, claw1, claw2;
    public VoltageSensor batteryVoltageSensor;
    private final List<LynxModule> hubs;

    public AngleSubsystem angle;

    public ArmSubsystem a;
    public ClawSubsystem claw;
    public SlidesSubsystem slidesSubsystem;

    public DriveSubsystem driveSubsystem;
    public MecanumDrive drive;
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
        arm.setDirection(DcMotor.Direction.REVERSE);

        linear_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linear_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        linear_2.setDirection(DcMotor.Direction.REVERSE);
        linear_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linear_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //servos
        Servo dump = hardwareMap.get(Servo.class, "dump");
        Servo claw1 = hardwareMap.get(Servo.class, "claw");
        Servo claw2 = hardwareMap.get(Servo.class, "claw1");


        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        for (LynxModule hub : hubs = hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        //setting subsystems
        a = new ArmSubsystem(arm, batteryVoltageSensor);
        claw = new ClawSubsystem(hardwareMap, "claw", "claw1");
        angle = new AngleSubsystem(hardwareMap, "dump");
        driveSubsystem = new DriveSubsystem(new SampleMecanumDrive(hardwareMap), false);
        CommandScheduler.getInstance().registerSubsystem(a, claw, angle, driveSubsystem);

    }

    public void currentUpdate(Telemetry telemetry) {
        List<Double> current_list = new ArrayList<>();
        current_list.add(leftFront.getCurrent(CurrentUnit.AMPS));
        current_list.add(rightFront.getCurrent(CurrentUnit.AMPS));
        current_list.add(leftRear.getCurrent(CurrentUnit.AMPS));
        current_list.add(rightRear.getCurrent(CurrentUnit.AMPS));
        current_list.add(arm.getCurrent(CurrentUnit.AMPS));
        current_list.add(linear_1.getCurrent(CurrentUnit.AMPS));
        current_list.add(linear_2.getCurrent(CurrentUnit.AMPS));

        telemetry.addLine(String.format(Locale.ENGLISH, "left: %.2f, %.2f, %.2f right: %.2f, %.2f, %.2f intake: %.2f arm: %.2f",
                current_list.get(0),
                current_list.get(1),
                current_list.get(2),
                current_list.get(3),
                current_list.get(4),
                current_list.get(5),
                current_list.get(6)
        ));

        double current = 0;

        for (LynxModule hub : hubs) {
            current += hub.getCurrent(CurrentUnit.AMPS);
        }

        telemetry.addData("Total Current: ", current);
    }

}