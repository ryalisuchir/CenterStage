package org.firstinspires.ftc.teamcode.Drive.OpModes.Worlds;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Utility.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.Vision.Robot.RedRobot;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class SensedRobotTest extends OpMode {
    private VisionPortal visionPortal;
    private RedRobot colorMassDetectionProcessor;
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

        colorMassDetectionProcessor = new RedRobot();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .addProcessor(colorMassDetectionProcessor)
                .build();
    }

    @Override
    public void init_loop() {
        telemetry.addData("Robot Recorded: ", colorMassDetectionProcessor.getSensedBoolean());
        telemetry.addData("Camera State: ", visionPortal.getCameraState());
        CommandScheduler.getInstance().run();
    }

    @Override
    public void start() {
        //opmode initialize stuff
    }

    @Override
    public void loop() {
       if (colorMassDetectionProcessor.getSensedBoolean() == RedRobot.Sensed.TRUE) {
           telemetry.addData("Robot in the Way: ", "FALSE");
       } else if (colorMassDetectionProcessor.getSensedBoolean() == RedRobot.Sensed.FALSE) {
           telemetry.addData("Robot in the Way: ", "TRUE");
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