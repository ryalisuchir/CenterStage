package org.firstinspires.ftc.teamcode.Drive.OpModes.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class TeleOpX extends LinearOpMode {

    private DcMotor rightFront, rightRear, leftFront, leftRear, linear_1, linear_2, arm;
    private Servo dump, claw1, claw, drone, droneL;

    private AnalogInput analogInput;

    @Override
    public void runOpMode() {
        analogInput = hardwareMap.get(AnalogInput.class, "leftUSS");
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

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double clawToggle = 0;
        boolean currentleftBP;
        boolean prevleftBP;
        boolean currentRightBP;
        boolean prevRightBP;
        double dumpy = 0.151;
        double slidesSpeed = 1;
        double speed = 0.7;
        boolean droneReady = false;
        double slidesPosition;
        boolean OVERRIDE = false;
        double turnMulitplier = 1;

        dump.setDirection(Servo.Direction.REVERSE);
        claw1.setDirection(Servo.Direction.REVERSE);

        waitForStart();
        if (opModeIsActive()) {

            currentleftBP = gamepad1.left_bumper;
            currentRightBP = gamepad1.right_bumper;

            while (opModeIsActive()) {
                slidesPosition = (double)(linear_1.getCurrentPosition() + linear_2.getCurrentPosition()) / 2;
                prevleftBP = currentleftBP;
                prevRightBP = currentRightBP;
                currentleftBP = gamepad1.left_bumper;
                currentRightBP = gamepad1.right_bumper;

                //Telemetry:
                telemetry.addData("distanceee", analogInput.getVoltage());
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

                //Movement:
                if (clawToggle == 1.1 || clawToggle == 1.2) {
                    turnMulitplier = 1;
                    speed = 1.5;
                } else if (clawToggle == 0) {
                    speed = 0.8;
                    turnMulitplier = 0.7;
                } else {
                    speed = 0.8;
                    turnMulitplier = 0.8;
                }

                leftFront.setPower(speed * (((gamepad1.left_stick_y + turnMulitplier * gamepad1.right_stick_x) + -1.3 * gamepad1.left_stick_x)));
                leftRear.setPower(speed * (((gamepad1.left_stick_y + turnMulitplier * gamepad1.right_stick_x) + 1.3 * gamepad1.left_stick_x)));
                rightFront.setPower(speed * ((gamepad1.left_stick_y - turnMulitplier * gamepad1.right_stick_x + 1.3 * gamepad1.left_stick_x)));
                rightRear.setPower(speed * ((gamepad1.left_stick_y - turnMulitplier * gamepad1.right_stick_x + -1.3 * gamepad1.left_stick_x)));

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
                if (gamepad1.x && clawToggle == 2) {
                    claw.setPosition(0.65);
                    claw1.setPosition(0);
                } else if (gamepad1.y && clawToggle == 2) {
                    claw.setPosition(0);
                    claw1.setPosition(0.655);
                } else if (gamepad1.x) {
                    claw.setPosition(0.68);
                    claw1.setPosition(0);
                } else if (gamepad1.y) {
                    claw.setPosition(0);
                    claw1.setPosition(0.68);
                } else if (gamepad1.right_trigger > 0 && (clawToggle == 0 || clawToggle == 1.1 || clawToggle == 1.2)) {
                    claw.setPosition(0.9);
                    claw1.setPosition(0.9);
                } else if (gamepad1.right_trigger > 0 && clawToggle == 2) {
                    if (gamepad1.right_trigger >= 0.8) {
                        claw.setPosition(gamepad1.right_trigger - 0.3);
                        claw1.setPosition(gamepad1.right_trigger - 0.3);
                    } else {
                        claw.setPosition(0.65);
                        claw1.setPosition(0.655);
                    }
                } else {
                    claw.setPosition(0.3);
                    claw1.setPosition(0.3);
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

                if (gamepad1.dpad_right) {
                    arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    linear_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    linear_1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    linear_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    linear_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }

        /* Arm Toggle Controls:
           NOTE: clawToggle {0 = ground, 1.1/1.2 = resting pos in middle, 2 = raised claw, 5 = hanging}
         */
                if (gamepad1.dpad_down) {
                    arm.setPower(.6);
                    dumpy = 0;
                    clawToggle = 5;
                }
                if (gamepad1.dpad_up && clawToggle == 1.1) {
                    dumpy = 0.151;
                    clawToggle = 0;

                }
                if (gamepad1.dpad_up && clawToggle == 1.2) {
                    arm.setPower(.6);
                    dumpy = 0.57;
                    clawToggle = 2;

                }
                if ((currentleftBP && !prevleftBP) && clawToggle == 0) {
                    dumpy = 0.4;
                    clawToggle = 1.1;
                } else if ((currentleftBP && !prevleftBP) && (clawToggle == 1.2 || clawToggle == 1.1 || clawToggle == 5)) {
                    arm.setPower(.6);
                    dumpy = 0.57;
                    clawToggle = 2;

                } else if ((currentRightBP && !prevRightBP) && (clawToggle == 1.2 || clawToggle == 1.1)) {
                    dumpy = 0.151;
                    clawToggle = 0;

                } else if ((currentRightBP && !prevRightBP) && clawToggle == 2) {
                    arm.setPower(-.57);
                    dumpy = 0.4;
                    clawToggle = 1.2;
                }
                if (gamepad1.left_trigger > 0) {
                    if (clawToggle == 0)
                        dump.setPosition(dumpy + (0.1) * gamepad1.left_trigger);
                    else
                        dump.setPosition(dumpy - (0.1) * gamepad1.left_trigger);
                } else dump.setPosition(dumpy);

                //0.59 is a good curve.
                if (gamepad2.dpad_down) {
                    droneL.setPosition(0.98);
                    droneReady = false;
                } else if (gamepad2.dpad_up) {
                    droneL.setPosition(0.7);
                    drone.setPosition(0.77);
                    droneReady = true;
                }
                if (gamepad2.cross && droneReady) {
                    drone.setPosition(0.4);
                }
                //Miscellaneous Speed Controls / Overrides:
                OVERRIDE = gamepad1.dpad_left;
            }
        }
    }
}