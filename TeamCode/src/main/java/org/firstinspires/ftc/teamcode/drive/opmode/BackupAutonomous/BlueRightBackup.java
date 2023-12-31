package org.firstinspires.ftc.teamcode.drive.opmode.BackupAutonomous;

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
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.ColorPropDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Scalar;

@Autonomous
public class BlueRightBackup extends OpMode {
    private VisionPortal visionPortal;
    private ColorPropDetectionProcessor colorMassDetectionProcessor;

    private Robot robot;
    private double loop;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        robot = new Robot(hardwareMap);

        CommandScheduler.getInstance().registerSubsystem(robot.a);
        CommandScheduler.getInstance().registerSubsystem(robot.claw);
        CommandScheduler.getInstance().registerSubsystem(robot.angle);
        CommandScheduler.getInstance().registerSubsystem(robot.driveSubsystem);

        telemetry.addData("Successful: ", "Ready for BlueRight (Backdrop Side)");
        telemetry.addData("Running: ", "1 pixel autonomous. All subsystems will run.");
        telemetry.update();

        robot.claw.grabBoth();
        Scalar lower = new Scalar(80, 50, 50);
        Scalar upper = new Scalar(180, 255, 255);
        double minArea = 100;

        colorMassDetectionProcessor = new ColorPropDetectionProcessor(
                lower,
                upper,
                () -> minArea,
                () -> 213, //left third
                () -> 426 //right third
        );
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .addProcessor(colorMassDetectionProcessor)
                .build();

    }

    @Override
    public void init_loop() {
        telemetry.addData("Currently Recorded Position", colorMassDetectionProcessor.getRecordedPropPosition());
        telemetry.addData("Camera State", visionPortal.getCameraState());
        telemetry.addData("Currently Detected Mass Center", "x: " + colorMassDetectionProcessor.getLargestContourX() + ", y: " + colorMassDetectionProcessor.getLargestContourY());
        telemetry.addData("Currently Detected Mass Area", colorMassDetectionProcessor.getLargestContourArea());
        CommandScheduler.getInstance().run();
        robot.a.loop();
    }

    @Override
    public void start() {
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.stopLiveView();
            visionPortal.stopStreaming();
        }

        ColorPropDetectionProcessor.PropPositions recordedPropPosition = colorMassDetectionProcessor.getRecordedPropPosition();

        if (recordedPropPosition == ColorPropDetectionProcessor.PropPositions.UNFOUND) {
            recordedPropPosition = ColorPropDetectionProcessor.PropPositions.MIDDLE;
        }

        switch (recordedPropPosition) {
            case LEFT:
            case UNFOUND:
                TrajectorySequence dropPixelLeft = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-39.26, 65.04, Math.toRadians(-90.00)))
                        .splineTo(
                                new Vector2d(-28.3, 36.83), Math.toRadians(0.00),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToConstantHeading(new Vector2d(-42.92, 36.13))
                        .build();

                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequencenotAsync(dropPixelLeft)),
                                new WaitCommand(1000),
                                new ParallelCommandGroup(
                                        new InstantCommand(() -> robot.a.armIntake()),
                                        new InstantCommand(() -> robot.angle.intake())
                                )
                        )
                );
                break;
            case MIDDLE:
                TrajectorySequence dropPixelMiddle = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-39.26, 65.04, Math.toRadians(270.00)))
                        .splineToConstantHeading(
                                new Vector2d(-35.96, 35.26), Math.toRadians(-90.00),
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToConstantHeading(new Vector2d(-35.78, 41.35))
                        .build();

                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequencenotAsync(dropPixelMiddle)),
                                new WaitCommand(1000),
                                new ParallelCommandGroup(
                                        new InstantCommand(() -> robot.a.armIntake()),
                                        new InstantCommand(() -> robot.angle.intake())
                                )
                        )
                );
                break;
            case RIGHT:
                TrajectorySequence dropPixelRight = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-39.26, 65.04, Math.toRadians(270.00)))
                        .splineToConstantHeading(
                                new Vector2d(-55.11, 41.53), Math.toRadians(-90.00),
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToConstantHeading(new Vector2d(-44.49, 53.72))
                        .build();

                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequencenotAsync(dropPixelRight)),
                                new WaitCommand(350),
                                new ParallelCommandGroup(
                                        new InstantCommand(() -> robot.a.armIntake()),
                                        new InstantCommand(() -> robot.angle.intake())
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
        telemetry.addData("Loop: ", time - loop);
        telemetry.addData("Arm Position: ", robot.a.getCachePos());
        loop = time;

        telemetry.update();
    }

    @Override
    public void stop() {
        colorMassDetectionProcessor.close();
        visionPortal.close();
        CommandScheduler.getInstance().reset();
    }
}