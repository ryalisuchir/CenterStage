package org.firstinspires.ftc.teamcode.drive.opmode.CompetitionAutonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.common.commandbase.command.GrabBothCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.RestCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.TapeDropperCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.common.commandbase.command.OuttakeCommand;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.ColorPropDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

@Autonomous
@Config
public class BlueLeft extends OpMode {
    private VisionPortal visionPortal;
    private ColorPropDetectionProcessor colorMassDetectionProcessor;

    private Robot robot;
    private ElapsedTime time_since_start;
    private double loop;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        robot = new Robot(hardwareMap);

        CommandScheduler.getInstance().registerSubsystem(robot.a);
        CommandScheduler.getInstance().registerSubsystem(robot.claw);
        CommandScheduler.getInstance().registerSubsystem(robot.angle);
        CommandScheduler.getInstance().registerSubsystem(robot.driveSubsystem);

        telemetry.addData("Not Ready: ", "Not able to proceed to camera detection... Restart robot now.");
        telemetry.update();

        robot.claw.grabBoth();

        Scalar lower = new Scalar(80, 50, 50);
        Scalar upper = new Scalar(180, 255, 255);
        double minArea = 100;

        colorMassDetectionProcessor = new ColorPropDetectionProcessor(
                lower,
                upper,
                () -> minArea,
                () -> 213, //left third of frame
                () -> 426 //right third of frame
        );
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .addProcessor(colorMassDetectionProcessor)
                .build();
    }

    @Override
    public void init_loop() {
        telemetry.addData("Successful: ", "Ready for BlueLeft (Backdrop Side)");
        telemetry.addData("Ready to Run: ", "2 pixel autonomous. All subsystems initialized.");
        telemetry.addData("Currently Recorded Position", colorMassDetectionProcessor.getRecordedPropPosition());
        telemetry.addData("Camera State", visionPortal.getCameraState());
        telemetry.addData("Currently Detected Mass Center", "x: " + colorMassDetectionProcessor.getLargestContourX() + ", y: " + colorMassDetectionProcessor.getLargestContourY());
        telemetry.addData("Currently Detected Mass Area", colorMassDetectionProcessor.getLargestContourArea());
        CommandScheduler.getInstance().run();
        robot.a.loop();
    }

    @Override
    public void start() {
        time_since_start = new ElapsedTime();
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.stopLiveView();
            visionPortal.stopStreaming();
        }

        ColorPropDetectionProcessor.PropPositions recordedPropPosition = colorMassDetectionProcessor.getRecordedPropPosition();

        switch (recordedPropPosition) {
            case LEFT:
            case RIGHT:
            case MIDDLE:
            case UNFOUND:
                TrajectorySequence OGbackBoardPixelLeft = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(18.89, 66.78, Math.toRadians(-90.00)))
                        .splineToSplineHeading(new Pose2d(49.19, 49.71, Math.toRadians(20.00)), Math.toRadians(20.00))
                        .build();

                TrajectorySequence tapePixelLeft = robot.driveSubsystem.trajectorySequenceBuilder(OGbackBoardPixelLeft.end())
                        .lineToConstantHeading(new Vector2d(38.05, 33.34))
                        .build();

                TrajectorySequence parkLeft = robot.driveSubsystem.trajectorySequenceBuilder(tapePixelLeft.end())
                        .lineToConstantHeading(new Vector2d(39.96, 76.00))
                        .lineToConstantHeading(new Vector2d(50, 74.96))
                        .build();

                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new WaitCommand(200),
                                new ParallelCommandGroup(
                                        new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequencenotAsync(OGbackBoardPixelLeft)),
                                        new OuttakeCommand(robot)
                                ),
                                new WaitCommand(2500),
                                new InstantCommand(() -> robot.claw.releaseRight()),
                                new WaitCommand(1000),
                                new ParallelCommandGroup(
                                        new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequencenotAsync(tapePixelLeft)),
                                        new TapeDropperCommand(robot)
                                ),
                                new WaitCommand(350),
                                new InstantCommand(() -> robot.claw.releaseLeft()),
                                new WaitCommand(350),
                                new RestCommand(robot),
                                new WaitCommand(1000),
                                new GrabBothCommand(robot),
                                new WaitCommand(1000),
                                new ParallelCommandGroup(
                                        new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequencenotAsync(parkLeft)),
                                        new OuttakeCommand(robot)
                                )
                        )
                );
                break;
        }
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        robot.a.loop();
        robot.driveSubsystem.update();

        double time = System.currentTimeMillis();
        telemetry.addData("Time Elapsed: ", time_since_start);
        telemetry.addData("Current Loop Time: ", time - loop);
        telemetry.addData("Arm Position: ", robot.a.getCachePos());
        robot.currentUpdate(telemetry);
        loop = time;
        telemetry.update();
    }

    @Override
    public void stop() {
        colorMassDetectionProcessor.close();
        visionPortal.close();
        telemetry.addLine("Closed Camera.");
        telemetry.update();
        CommandScheduler.getInstance().reset();
    }
}