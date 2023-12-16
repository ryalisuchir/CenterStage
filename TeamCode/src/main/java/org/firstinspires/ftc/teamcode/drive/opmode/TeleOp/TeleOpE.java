package org.firstinspires.ftc.teamcode.drive.opmode.TeleOp;//package org.firstinspires.ftc.teamcode.drive.opmode.Autonomous;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.Arm;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.ArmSubsystem;

@TeleOp
public class TeleOpE extends LinearOpMode {


    public DcMotorEx armor;

    private VoltageSensor batteryVoltageSensor;


    private final double p = 0.01; //adjust TODO
    private final double d = 0.000834; //adjust TODO
    private final double f = 0.008; //adjust TODO
    private final double ticks_to_degrees = 384.5 / 180.0;

    private PIDController controller;
    private ElapsedTime time;
    private ElapsedTime voltageTimer;
    private double voltage;

    private MotionProfile profile;
    public static double max_v = 10000;
    public static double max_a = 6000;

    private int target = 5;
    private int previous_target = 5;


    private double cache = 0;



    private DcMotor rightFront;
    private DcMotor rightRear;
    private DcMotor linear_2;
    private Servo dump;
    private Servo claw1;

    private Servo drone;
    private DcMotor leftFront;
    private DcMotor leftRear;
    private DcMotor linear_1;
    private Servo claw;


    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        armor = hardwareMap.get(DcMotorEx.class, "arm");

        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        linear_2 = hardwareMap.get(DcMotor.class, "linear_2");
        dump = hardwareMap.get(Servo.class, "dump");
        drone = hardwareMap.get(Servo.class, "drone");
        claw1 = hardwareMap.get(Servo.class, "claw1");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        linear_1 = hardwareMap.get(DcMotor.class, "linear_1");
        claw = hardwareMap.get(Servo.class, "claw");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        linear_2.setDirection(DcMotor.Direction.REVERSE);
        double intakeSpeed = 0;
        double slidesSpeed = 0.5;
        double speed = 0.5;
        dump.setDirection(Servo.Direction.REVERSE);
        claw1.setDirection(Servo.Direction.REVERSE);
        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {

                // Put loop blocks here.
                telemetry.update();
                leftFront.setPower(speed * (((gamepad1.left_stick_y - gamepad1.right_stick_x) + -1 * gamepad1.left_stick_x) / 1));
                leftRear.setPower(speed * (((gamepad1.left_stick_y - gamepad1.right_stick_x) + 1 * gamepad1.left_stick_x) / 1));
                rightFront.setPower(speed * ((gamepad1.left_stick_y + gamepad1.right_stick_x + 1 * gamepad1.left_stick_x) / 1));
                rightRear.setPower(speed * ((gamepad1.left_stick_y + gamepad1.right_stick_x + -1 * gamepad1.left_stick_x) / 1));
                if (gamepad1.a) {
                    intakeSpeed += 0.05;
                    telemetry.addData("intakeSpeed", intakeSpeed);
                    sleep(100);
                }
                if (gamepad1.b) {
                    intakeSpeed -= 0.05;
                    telemetry.addData("intakeSpeed", intakeSpeed);
                    sleep(100);
                }
                linear_1.setPower(slidesSpeed * gamepad2.left_stick_y);
                linear_2.setPower(slidesSpeed * gamepad2.left_stick_y);
                if (gamepad2.dpad_up) {
                    slidesSpeed = 3;
                } else if (gamepad2.dpad_down) {
                    slidesSpeed = 0.5;
                }
                claw1.setPosition(gamepad1.right_trigger );
                claw.setPosition(gamepad1.right_trigger );
                dump.setPosition(intakeSpeed);

                if (gamepad1.dpad_up) {
                    target = 200;
                    intakeSpeed = 1;
                }else if (gamepad1.dpad_down) {
                    target = 0;
                    intakeSpeed = 0;
                }
                if (gamepad2.dpad_left)
                    drone.setPosition(0);
                else if (gamepad2.dpad_right)
                    drone.setPosition(1);
            }
        }
    }
}