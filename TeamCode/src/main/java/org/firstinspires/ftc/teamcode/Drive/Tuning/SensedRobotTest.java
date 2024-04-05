package org.firstinspires.ftc.teamcode.Drive.Tuning;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Utility.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.Vision.Robot.Wall.BlueWallRobotScan;
import org.firstinspires.ftc.vision.VisionPortal;

//This OpMode can be used for both blue or red! Just change the naming.
@Autonomous
public class SensedRobotTest extends OpMode {
    private VisionPortal visionPortal;
    private BlueWallRobotScan colorMassDetectionProcessor;
    private RobotHardware robot;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        robot = new RobotHardware(hardwareMap);

        CommandScheduler.getInstance().registerSubsystem(robot.armSystem);
        CommandScheduler.getInstance().registerSubsystem(robot.claw);
        CommandScheduler.getInstance().registerSubsystem(robot.angleOfArm);
        CommandScheduler.getInstance().registerSubsystem(robot.driveSubsystem);

        telemetry.addData("Not Ready: ", "Not able to proceed to camera detection... Restart robot now.");
        telemetry.update();

        robot.claw.grabBoth();

        colorMassDetectionProcessor = new BlueWallRobotScan();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .addProcessor(colorMassDetectionProcessor)
                .build();
    }

    @Override
    public void init_loop() {
        telemetry.addData("Robot Recorded: ", colorMassDetectionProcessor.getSensedBoolean());
        telemetry.addData("Camera State: ", visionPortal.getCameraState());
        telemetry.addData("Sensed Percent: ", colorMassDetectionProcessor.getSensedPercent());
        CommandScheduler.getInstance().run();
    }

    @Override
    public void start() {
        //opmode initialize stuff
    }

    @Override
    public void loop() {
       if (colorMassDetectionProcessor.getSensedBoolean() == BlueWallRobotScan.Sensed.TRUE) {
           telemetry.addData("Robot in the Way: ", "TRUE");
       } else if (colorMassDetectionProcessor.getSensedBoolean() == BlueWallRobotScan.Sensed.FALSE) {
           telemetry.addData("Robot in the Way: ", "FALSE");
       } else {
           telemetry.addLine("Unable to detect.");
       }
       telemetry.update();
    }

    @Override
    public void stop() {
        visionPortal.close();
        telemetry.addLine("Closed Camera.");
        telemetry.update();
        CommandScheduler.getInstance().reset();
    }

}