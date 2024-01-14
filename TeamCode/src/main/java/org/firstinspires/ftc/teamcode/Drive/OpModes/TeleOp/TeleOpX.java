package org.firstinspires.ftc.teamcode.Drive.OpModes.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class TeleOpX extends LinearOpMode {

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
    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

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
        int clawToggle = 0;
        double dumpy = 0;
        double slidesSpeed = 0.5;
        double speed = 0.5;
        dumpy = 0.05;

        slidesSpeed = 0.5;
        speed = 0.65;

        dump.setDirection(Servo.Direction.REVERSE);
        claw1.setDirection(Servo.Direction.REVERSE);

        waitForStart();
        if (opModeIsActive()) {

            while (opModeIsActive()) {

                //telemetry
                telemetry.addData("armPos", arm.getCurrentPosition());
                telemetry.addData("armPow", arm.getPower());
                telemetry.addData("dronePos", drone.getPosition());
                telemetry.addData("droneLPos", droneL.getPosition());
                telemetry.addData("dumpy", dumpy);
                telemetry.addData("dumpPos", dump.getPosition());
                telemetry.addData("clawToggle", clawToggle);
                telemetry.addData("slidesPos", (linear_1.getCurrentPosition() + linear_2.getCurrentPosition()) / 2);
                telemetry.update();

                //movement
                leftFront.setPower(speed * (((gamepad1.left_stick_y + gamepad1.right_stick_x) + -1 * gamepad1.left_stick_x) / 1));
                leftRear.setPower(speed * (((gamepad1.left_stick_y + gamepad1.right_stick_x) + 1 * gamepad1.left_stick_x) / 1));
                rightFront.setPower(speed * ((gamepad1.left_stick_y - gamepad1.right_stick_x + 1 * gamepad1.left_stick_x) / 1));
                rightRear.setPower(speed * ((gamepad1.left_stick_y - gamepad1.right_stick_x + -1 * gamepad1.left_stick_x) / 1));

                //slides control
                linear_1.setPower(slidesSpeed * gamepad2.right_stick_y);
                linear_2.setPower(slidesSpeed * gamepad2.right_stick_y);
                if (gamepad2.right_bumper)
                    slidesSpeed = 0.8;
                else if (gamepad2.left_bumper)
                    slidesSpeed = 0.5;

                //claw control
                if (gamepad1.right_trigger > 0) {
                    claw.setPosition(1);
                    claw1.setPosition(1);
                } else {
                    claw.setPosition(0);
                    claw1.setPosition(0);
                }
                if (gamepad1.right_trigger > 0 && clawToggle == 0)
                    dumpy = 0.01;
                else if (gamepad1.right_trigger == 0 && clawToggle == 0)
                    dumpy = 0.01;

                // conditionals for arm
                if ((arm.getPower() > 0 && distance.getDistance(DistanceUnit.CM) < 3) || (arm.getCurrentPosition() < 20 && arm.getPower() < 0)) {
                    arm.setPower(0);
                }
                if (arm.getPower() < 0 && arm.getCurrentPosition() < 300) {
                    arm.setPower(0.0006);
                }
                if (arm.getPower() > 0 && arm.getCurrentPosition() > 300) {
                    arm.setPower(0.13);
                }

                //arm encoder reset
                if (gamepad1.x)
                    arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /* arm toggling controls
           NOTE: clawToggle {0 = ground, 1 = resting pos in middle, 2 = raised claw, 5 = hanging}
         */
                if (gamepad1.dpad_down) {
                    arm.setPower(.35);
                    dumpy = 0;
                    clawToggle = 5;
                }

                if (gamepad1.left_bumper && clawToggle == 0) {
                    dumpy = 0.3;
                    clawToggle = 1;
                    sleep(250);
                }
                if (gamepad1.left_bumper && (clawToggle == 1 || clawToggle == 5)) {
                    arm.setPower(.35);
                    dumpy = 0.6;
                    clawToggle = 2;
                    dump.setPosition(dumpy);
                    sleep(250);
                }
                if (gamepad1.right_bumper && clawToggle == 1) {
                    dumpy = 0.01;
                    clawToggle = 0;
                    dump.setPosition(dumpy);
                    sleep(250);
                }
                if (gamepad1.right_bumper && clawToggle == 2) {
                    arm.setPower(-.35);
                    dumpy = 0.3;
                    clawToggle = 1;
                    sleep(250);
                }
                dump.setPosition(dumpy);

                //drone controls 0.7-0......0.95-0.65
                if (gamepad2.dpad_down) {
                    droneL.setPosition(0.92);
                } else if (gamepad2.dpad_up) {
                    droneL.setPosition(0.68);
                    drone.setPosition(0.75);
                }
                if (gamepad2.x && droneL.getPosition() < 7) {
                    drone.setPosition(0);
                }
                //miscellaneous speed controls
            }
        }
    }
}