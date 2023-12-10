package org.firstinspires.ftc.teamcode.drive.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(group = "drive")
@Disabled
public class Values extends LinearOpMode {
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;
    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addData("Left Front Position: ", leftFront.getCurrentPosition());
            telemetry.addData("Right Front Position: ", rightFront.getCurrentPosition());
            telemetry.addData("Left Rear Position: ", leftRear.getCurrentPosition());
            telemetry.addData("Right Rear Position: ", rightRear.getCurrentPosition());
            telemetry.update();
        }
    }
}
