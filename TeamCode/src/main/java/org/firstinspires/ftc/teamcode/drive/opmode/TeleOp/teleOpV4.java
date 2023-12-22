
package org.firstinspires.ftc.teamcode.drive.opmode.TeleOp;//package org.firstinspires.ftc.teamcode.drive.opmode.Autonomous;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "teleOpV4-suchircomputer")
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
    private Servo droneL;

    private Servo drone;
    private DcMotor leftFront;
    private DcMotor leftRear;
    private DcMotor linear_1;
    private DcMotor arm;
    private Servo claw;


    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

        double dumpy1 = 0;
        double dumpy;
        double slidesSpeed;
        double speed;
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

        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        linear_2.setDirection(DcMotor.Direction.REVERSE);
        dumpy = 0.75;
        slidesSpeed = 0.5;
        speed = 0.5;
        dump.setDirection(Servo.Direction.REVERSE);
        claw1.setDirection(Servo.Direction.REVERSE);
        // Put initialization blocks here.ffvvvvv
        droneL.setPosition(0.7);
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {


                // Put loop blocks here.
                telemetry.update();
                leftFront.setPower(speed * (((gamepad1.left_stick_y + gamepad1.right_stick_x) + -1 * gamepad1.left_stick_x) / 1));
                leftRear.setPower(speed * (((gamepad1.left_stick_y + gamepad1.right_stick_x) + 1 * gamepad1.left_stick_x) / 1));
                rightFront.setPower(speed * ((gamepad1.left_stick_y - gamepad1.right_stick_x + 1 * gamepad1.left_stick_x) / 1));
                rightRear.setPower(speed * ((gamepad1.left_stick_y - gamepad1.right_stick_x + -1 * gamepad1.left_stick_x) / 1));
                arm.setPower(gamepad2.right_trigger - gamepad2.left_trigger);


               /*
               leftFront.setPower(speed * (((gamepad1.left_stick_y - gamepad1.right_stick_x) + -1 * gamepad1.left_stick_x) / 1));
                leftRear.setPower(speed * (((gamepad1.left_stick_y - gamepad1.right_stick_x) + 1 * gamepad1.left_stick_x) / 1));
                rightFront.setPower(speed * ((gamepad1.left_stick_y + gamepad1.right_stick_x + 1 * gamepad1.left_stick_x) / 1));
                rightRear.setPower(speed * ((gamepad1.left_stick_y + gamepad1.right_stick_x + -1 * gamepad1.left_stick_x) / 1));
                arm.setPower(gamepad2.right_trigger - gamepad2.left_trigger);


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
                linear_1.setPower(slidesSpeed * gamepad2.left_stick_y);
                linear_2.setPower(slidesSpeed * gamepad2.left_stick_y);
                if (gamepad2.dpad_up) {
                    slidesSpeed = 3;
                } else if (gamepad2.dpad_down) {
                    slidesSpeed = 0.5;
                }


                  /*  if (gamepad1.left_stick_y < -.3) {
                        linear_1.setPower(-.3);
                        linear_2.setPower(-.3);
                        arm.setPower(-.8);
                        dumpy = 0;
                        dump.setPosition(dumpy);
                    }
                } else {
                    sleep(100);
                    arm.setPower(0);

                   */

                telemetry.addData("Position: ", drone.getPosition());
                telemetry.addData("Position launch: ", droneL.getPosition());
                if (gamepad1.y)
                    dump.setPosition(0.56);

                if ( gamepad1.left_bumper) {
                    dumpy = 0.7;
                    dump.setPosition(dumpy);
                }
                if (gamepad1.right_bumper) {
                    dumpy = 0;
                    dump.setPosition(dumpy);
                }
                    if (gamepad2.dpad_left) {
                        drone.setPosition(0);
                        //droneL.setPosition(0);
                    } //0.9 when drone at idal launch 0.75 when dronel is at ideal launch
                    else if (gamepad2.dpad_right) {
                        //  droneL.setPosition(0.5);
                        //   sleep(250);ddddrrrrrr
                        drone.setPosition(0.9);
                    }
                    if (gamepad2.a && drone.getPosition() >0)
                        droneL.setPosition(1);


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

            }
        }
    }
}