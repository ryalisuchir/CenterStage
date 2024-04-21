package org.firstinspires.ftc.teamcode.Drive.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
@Config
public class ServoTuner extends OpMode {
    public static double servoPos = 0.39;
    Servo dumpster, claw;
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        dumpster = hardwareMap.get(Servo.class, "dump");
        claw = hardwareMap.get(Servo.class, "claw1");
    }

    @Override
    public void loop() {
        dumpster.setPosition(0.450);
        claw.setPosition(servoPos);
        telemetry.update();
    }
}
