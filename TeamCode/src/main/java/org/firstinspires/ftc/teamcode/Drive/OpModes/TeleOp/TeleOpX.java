package org.firstinspires.ftc.teamcode.Drive.OpModes.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TeleOpX extends LinearOpMode {

    private DcMotor leftFront, leftRear, rightFront, rightRear, linear_1, linear_2, arm;
    private Servo claw, claw1, dump, drone, droneL;
    double clawToggle, dumpy, slidesSpeed, speed, slidesPosition, turnMultiplier;
    boolean currentLeftBP, prevLeftBP, currentRightBP, prevRightBP, droneReady, OVERRIDE;

    @Override
    public void runOpMode() {

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
        linear_2.setDirection(DcMotor.Direction.REVERSE);
        linear_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear_1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linear_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setDirection(DcMotor.Direction.REVERSE);

        dump.setDirection(Servo.Direction.REVERSE);
        claw1.setDirection(Servo.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawToggle = 0;
        dumpy = 0.151;
        slidesSpeed = 1;
        speed = 0.7;
        droneReady = false;
        OVERRIDE = false;
        turnMultiplier = 1;

        waitForStart();
        if (opModeIsActive()) {
            currentLeftBP = gamepad1.left_bumper;
            currentRightBP = gamepad1.right_bumper;

            while (opModeIsActive()) {
                slidesPosition = (double)(linear_1.getCurrentPosition() + linear_2.getCurrentPosition()) / 2;
                prevLeftBP = currentLeftBP;
                prevRightBP = currentRightBP;
                currentLeftBP = gamepad1.left_bumper;
                currentRightBP = gamepad1.right_bumper;

                telemetry.addData("clawToggle", clawToggle);
                telemetry.addData("armPos", arm.getCurrentPosition());
                telemetry.addData("armPow", arm.getPower());
                telemetry.addData("dronePos", drone.getPosition());
                telemetry.addData("droneLPos", droneL.getPosition());
                telemetry.addData("dumpy", dumpy);
                telemetry.addData("dumpPos", dump.getPosition());
                telemetry.addData("slidesPos", slidesPosition);
                telemetry.addData("slidesPow", (gamepad2.right_trigger));
                telemetry.addData("speed", speed);
                telemetry.update();

                if (clawToggle == 1.1 || clawToggle == 1.2) {
                    turnMultiplier = 1;
                    speed = 1;
                } else if (clawToggle == 0) {
                    speed = 0.9;
                    turnMultiplier = 0.8;
                } else {
                    speed = 0.8;
                    turnMultiplier = 0.8;
                }

                leftFront.setPower(speed * (((gamepad1.left_stick_y + turnMultiplier * gamepad1.right_stick_x) + -1.3 * gamepad1.left_stick_x)));
                leftRear.setPower(speed * (((gamepad1.left_stick_y + turnMultiplier * gamepad1.right_stick_x) + 1.3 * gamepad1.left_stick_x)));
                rightFront.setPower(speed * ((gamepad1.left_stick_y - turnMultiplier * gamepad1.right_stick_x + 1.3 * gamepad1.left_stick_x)));
                rightRear.setPower(speed * ((gamepad1.left_stick_y - turnMultiplier * gamepad1.right_stick_x + -1.3 * gamepad1.left_stick_x)));

                //Slides Control:
                if ((-gamepad2.right_stick_y) < 0 && slidesPosition >= -15 && !OVERRIDE) {
                    linear_1.setPower(0);
                    linear_2.setPower(0);
                } else if ((-gamepad2.right_stick_y) == 0 && slidesPosition < -230) {
                    linear_1.setPower(0.06);
                    linear_2.setPower(0.06);
                } else {
                    linear_1.setPower(slidesSpeed * (-gamepad2.right_stick_y));
                    linear_2.setPower(slidesSpeed * (-gamepad2.right_stick_y));
                }

                if (gamepad2.right_bumper)
                    slidesSpeed = 1;
                else if (gamepad2.left_bumper)
                    slidesSpeed = 0.8;

                //Claw Control:
                if (gamepad1.x) {
                    claw.setPosition(0.7);
                    claw1.setPosition(0);
                } else if (gamepad1.y) {
                    claw.setPosition(0);
                    claw1.setPosition(0.7);
                } else if (gamepad1.right_trigger > 0 && (clawToggle == 0 || clawToggle == 1.1 || clawToggle == 1.2)) {
                    claw.setPosition(0.9);
                    claw1.setPosition(0.9);
                } else if (gamepad1.right_trigger > 0 && clawToggle == 2) {
                    if (gamepad1.right_trigger >= 0.4) {
                        claw.setPosition(gamepad1.right_trigger);
                        claw1.setPosition(gamepad1.right_trigger);
                    } else {
                        claw.setPosition(0.7);
                        claw1.setPosition(0.7);
                    }
                } else {
                    claw.setPosition(0);
                    claw1.setPosition(0);
                }

                //Arm Conditionals:
                if (!OVERRIDE) {
                    if ((arm.getPower() > 0 && arm.getCurrentPosition() > 430) || (arm.getCurrentPosition() < 20 && arm.getPower() < 0)) {
                        arm.setPower(0);
                    }
                    if (arm.getPower() < 0 && arm.getCurrentPosition() < 230) {
                        arm.setPower(0.0006);
                    }
                    if (arm.getPower() > 0 && arm.getCurrentPosition() > 350) {
                        arm.setPower(0.33);
                    }
                }

                //Arm / Slides Reset:
                if (gamepad1.dpad_right) {
                    arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    linear_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    linear_1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    linear_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    linear_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
                //Arm Toggle Controls:
                //clawToggle {0 = ground, 1.1/1.2 = est, 2 = raised claw, 5 = hanging}
                if (gamepad1.dpad_down) {
                    arm.setPower(.5);
                    dumpy = 0;
                    clawToggle = 5;
                }
                if (gamepad1.dpad_up && clawToggle == 1.1) {
                    dumpy = 0.151;
                    clawToggle = 0;

                }
                if (gamepad1.dpad_up && clawToggle == 1.2) {
                    arm.setPower(.53);
                    dumpy = 0.57;
                    clawToggle = 2;

                }
                if ((currentLeftBP && !prevLeftBP) && clawToggle == 0) {
                    dumpy = 0.4;
                    clawToggle = 1.1;
                } else if ((currentLeftBP && !prevLeftBP) && (clawToggle == 1.2 || clawToggle == 1.1 || clawToggle == 5)) {
                    arm.setPower(.53);
                    dumpy = 0.57;
                    clawToggle = 2;

                } else if ((currentRightBP && !prevRightBP) && (clawToggle == 1.2 || clawToggle == 1.1)) {
                    dumpy = 0.151;
                    clawToggle = 0;

                } else if ((currentRightBP && !prevRightBP) && clawToggle == 2) {
                    arm.setPower(-.53);
                    dumpy = 0.4;
                    clawToggle = 1.2;
                }
                if (gamepad1.left_trigger > 0) {
                    if (clawToggle == 0)
                        dump.setPosition(dumpy + (0.1) * gamepad1.left_trigger);
                    else
                        dump.setPosition(dumpy - (0.1) * gamepad1.left_trigger);
                } else dump.setPosition(dumpy);

                //0.59 worked well for drone curve
                if (gamepad2.dpad_down) {
                    droneL.setPosition(0.98);
                    droneReady = false;
                } else if (gamepad2.dpad_up) {
                    droneL.setPosition(0.70);
                    drone.setPosition(0.79);
                    droneReady = true;
                }
                if (gamepad2.cross && droneReady) {
                    drone.setPosition(0.4);
                }

                //Override Controls:
                OVERRIDE = gamepad1.dpad_left;
            }
        }
    }
}