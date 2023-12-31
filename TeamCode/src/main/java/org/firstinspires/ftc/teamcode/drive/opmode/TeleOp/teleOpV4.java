
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
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp
public class teleOpV4 extends LinearOpMode {


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
    private Servo droneL;
    private DcMotor leftFront;
    private DcMotor leftRear;
    private DcMotor linear_1;
    private DcMotor arm;
    private Servo claw;

    private TouchSensor touch;
    private DistanceSensor distance;
    //  private DistanceSensor distanceB;
    private Robot robot;

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        int clawToggle = 0;
        double armPWR = 0;
        double dumpy;
        double slidesSpeed;
        double speed;
        boolean armDrop = false;
        touch = hardwareMap.get(TouchSensor.class, "touch");
        distance = hardwareMap.get(DistanceSensor.class, "distance");
        //      distanceB = hardwareMap.get(DistanceSensor.class, "distanceBottom");

        arm = hardwareMap.get(DcMotor.class, "arm");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        linear_2 = hardwareMap.get(DcMotor.class, "linear_2");
        dump = hardwareMap.get(Servo.class, "dump");
        drone = hardwareMap.get(Servo.class, "drone");
        droneL = hardwareMap.get(Servo.class, "droneL");

        claw1 = hardwareMap.get(Servo.class, "claw1");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        linear_1 = hardwareMap.get(DcMotor.class, "linear_1");
        claw = hardwareMap.get(Servo.class, "claw");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        linear_1.setDirection(DcMotor.Direction.REVERSE);
        arm.setDirection(DcMotor.Direction.REVERSE);
        dumpy = 0;

        slidesSpeed = 0.5;
        speed = 0.5;
        dump.setDirection(Servo.Direction.REVERSE);
        claw1.setDirection(Servo.Direction.REVERSE);
        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            // arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // Put run blocks here.
            while (opModeIsActive()) {


                // Put loop blocks here.
                leftFront.setPower(speed * (((gamepad1.left_stick_y + gamepad1.right_stick_x) + -1 * gamepad1.left_stick_x) / 1));
                leftRear.setPower(speed * (((gamepad1.left_stick_y + gamepad1.right_stick_x) + 1 * gamepad1.left_stick_x) / 1));
                rightFront.setPower(speed * ((gamepad1.left_stick_y - gamepad1.right_stick_x + 1 * gamepad1.left_stick_x) / 1));
                rightRear.setPower(speed * ((gamepad1.left_stick_y - gamepad1.right_stick_x + -1 * gamepad1.left_stick_x) / 1));
               /*
                if (gamepad1.a) {
                    dumpy += 0.05;
                    telemetry.addData("intakeSpeed", dumpy);
                    sleep(100);
                }
                if (gamepad1.b) {
                    dumpy -= 0.05;
                    telemetry.addData("intakeSpeed", dumpy);
                    sleep(100);
                }

                */


                //  droneL.setPosition(gamepad2.right_stick_y);

                if (gamepad1.x) {
                    arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                linear_1.setPower(gamepad1.right_stick_y);
                linear_2.setPower(gamepad1.right_stick_y);
                if (gamepad1.dpad_up) {
                    slidesSpeed += 0.05;

                    sleep(200);
                } else if (gamepad1.dpad_down) {
                    slidesSpeed -= 0.05;

                    sleep(200);
                }
                claw1.setPosition(gamepad1.right_trigger);
                claw.setPosition(gamepad1.right_trigger);
                if (gamepad1.left_trigger > 0) {
                    dump.setPosition(dumpy - (0.4 * gamepad1.left_trigger));
                } else
                    dump.setPosition(dumpy);

                if(arm.getPower()>0 && distance.getDistance(DistanceUnit.CM) < 3) {
                    arm.setPower(0);
                    arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }

                if (arm.getCurrentPosition()>-300 && arm.getPower() > 0 && !armDrop) {
                    arm.setPower(0);

                }


                if (arm.getCurrentPosition()<-215 && arm.getPower() < 0) {
                    arm.setPower(0.03);

                }


                if (arm.getCurrentPosition() < -625 && arm.getPower() <= 0.04 && arm.getPower() >= 0.02) {
                    arm.setPower(0);
                }
                if (gamepad1.left_bumper) {
                    armDrop = false;
                    dumpy = 0.6;

                    arm.setPower(.35);
                    ;
                }
                if (gamepad1.right_bumper) {
                    arm.setPower(-.35);
                    dumpy = 0;
                    armDrop = true;

                }


                if (gamepad1.dpad_left) {
                    drone.setPosition(0.6);
                    //droneL.setPosition(0);
                } //0.9 when drone at idal launch 0.75 when dronel is at ideal launch
                else if (gamepad1.dpad_right) {
                    //  droneL.setPosition(0.5);
                    //   sleep(250);ddddrrrrrr
                    drone.setPosition(slidesSpeed);
                }
                if (gamepad1.x && drone.getPosition() >0.6)
                    droneL.setPosition(1);
                if (gamepad1.y && drone.getPosition() >0.6)
                    droneL.setPosition(0.8);

                if (gamepad1.a) {
                    claw.setPosition(gamepad1.right_trigger);
                    claw1.setPosition(0);
                    sleep(250);
                }
                if (gamepad1.b) {
                    claw1.setPosition(gamepad1.right_trigger);
                    claw.setPosition(0);
                    sleep(250);
                }
                if (!(gamepad1.a && gamepad1.b)) {

                    claw1.setPosition(gamepad1.right_trigger);
                    claw.setPosition(gamepad1.right_trigger);

                }
                telemetry.addData("Arm Position: ", arm.getCurrentPosition());
                telemetry.addData("Arm Power: ", arm.getPower());
                telemetry.addData("Slides Speed: ", slidesSpeed);
                telemetry.addData("Drone L Position: ", droneL.getPosition());
                robot.currentUpdate(telemetry); //don't remove this. this is tracking the voltage of each motor to prevent burnout.
                telemetry.update();
            }
        }
    }
}
