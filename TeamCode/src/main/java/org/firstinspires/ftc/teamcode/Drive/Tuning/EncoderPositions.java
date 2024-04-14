package org.firstinspires.ftc.teamcode.Drive.Tuning;

import com.acmerobotics.dashboard.config.Config;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
@Disabled
public class EncoderPositions extends LinearOpMode {
    private DcMotor leftFront, rightFront, leftRear, rightRear, linear_1, linear_2, arm;
    private Rev2mDistanceSensor distance;
    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        linear_1 = hardwareMap.get(DcMotor.class, "linear_1");
        linear_2 = hardwareMap.get(DcMotor.class, "linear_2");
        arm = hardwareMap.get(DcMotor.class, "arm");

        distance = hardwareMap.get(Rev2mDistanceSensor.class, "distance");

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linear_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linear_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        linear_1.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addData("Left Front Position: ", leftFront.getCurrentPosition());
            telemetry.addData("Right Front Position: ", rightFront.getCurrentPosition());
            telemetry.addData("Left Rear Position: ", leftRear.getCurrentPosition());
            telemetry.addData("Right Rear Position: ", rightRear.getCurrentPosition());
            telemetry.addData("Linear Slides 1 Position: ", linear_1.getCurrentPosition());
            telemetry.addData("Linear Slides 2 Position: ", linear_2.getCurrentPosition());
            telemetry.addData("Arm Position: ", arm.getCurrentPosition());
            telemetry.addData("Distance Sensor: ", distance.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}