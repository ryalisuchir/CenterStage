package org.firstinspires.ftc.teamcode.drive.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp(group = "drive")
@Disabled
public class EncoderPosition extends LinearOpMode {
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        waitForStart();

        while (!isStopRequested()) {

            telemetry.addData("right;", leftFront.getCurrentPosition());
            telemetry.addData("left:", rightFront.getCurrentPosition());
            telemetry.addData("strafe:", leftRear.getCurrentPosition());
            telemetry.update();
        }
    }
}
