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
public class RedRightBackup extends OpMode {
    private VisionPortal visionPortal;
    private ColorPropDetectionProcessor colourMassDetectionProcessor;

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

        telemetry.addData("Successful: ", "Ready for RedRight (Backdrop Side)");
        telemetry.addData("Running: ", "1 pixel autonomous. All subsystems will run.");
        telemetry.update();

        robot.claw.grabBoth();
        Scalar lower = new Scalar(0, 80, 80); // the lower hsv threshold
        Scalar upper = new Scalar(180, 250, 250); // the upper hsv threshold
        double minArea = 100; //min area for prop detection

        colourMassDetectionProcessor = new ColorPropDetectionProcessor(
                lower,
                upper,
                () -> minArea,
                () -> 213, //left third
                () -> 426 //right third
        );
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .addProcessor(colourMassDetectionProcessor)
                .build();

    }

    @Override
    public void init_loop() {
        telemetry.addData("Currently Recorded Position", colourMassDetectionProcessor.getRecordedPropPosition());
        telemetry.addData("Camera State", visionPortal.getCameraState());
        telemetry.addData("Currently Detected Mass Center", "x: " + colourMassDetectionProcessor.getLargestContourX() + ", y: " + colourMassDetectionProcessor.getLargestContourY());
        telemetry.addData("Currently Detected Mass Area", colourMassDetectionProcessor.getLargestContourArea());
        CommandScheduler.getInstance().run();
        robot.a.loop();
    }

    @Override
    public void start() {
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.stopLiveView();
            visionPortal.stopStreaming();
        }

        ColorPropDetectionProcessor.PropPositions recordedPropPosition = colourMassDetectionProcessor.getRecordedPropPosition();

        if (recordedPropPosition == ColorPropDetectionProcessor.PropPositions.UNFOUND) {
            recordedPropPosition = ColorPropDetectionProcessor.PropPositions.MIDDLE;
        }

        switch (recordedPropPosition) {
            case LEFT:
            case UNFOUND:
                TrajectorySequence dropPixelLeft = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(18.89, -66.78, Math.toRadians(90)))
                        .splineToSplineHeading(
                                new Pose2d(6.53, -36.30, Math.toRadians(180.00)), Math.toRadians(180.00),
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence backdropPixelLeft = robot.driveSubsystem.trajectorySequenceBuilder(dropPixelLeft.end())
                        .lineToConstantHeading(new Vector2d(16.45, -38.57))
                        .build();


                TrajectorySequence parkLeft = robot.driveSubsystem.trajectorySequenceBuilder(backdropPixelLeft.end())
                        .lineToSplineHeading(new Pose2d(67.65, -63.29, Math.toRadians(0.00)))
                        .build();


                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequencenotAsync(dropPixelLeft)),
                                new WaitCommand(1000),
                                new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequencenotAsync(backdropPixelLeft)),
                                new WaitCommand(1000),
                                new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequencenotAsync(parkLeft)),
                                new ParallelCommandGroup(
                                        new InstantCommand(() -> robot.a.armIntake()),
                                        new InstantCommand(() -> robot.angle.intake())
                                )
                        )
                );
                break;
            case MIDDLE:
                TrajectorySequence dropPixelMiddle = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(18.89, -66.78, Math.toRadians(90.00)))
                        .lineToConstantHeading(new Vector2d(19.59, -39.5))
                        .build();

                TrajectorySequence backdropPixelMiddle = robot.driveSubsystem.trajectorySequenceBuilder(dropPixelMiddle.end())
                        .lineToConstantHeading(new Vector2d(16.45, -46.06))
                        .build();


                TrajectorySequence parkMiddle = robot.driveSubsystem.trajectorySequenceBuilder(backdropPixelMiddle.end())
                        .lineToSplineHeading(new Pose2d(70, -78, Math.toRadians(0.00)))
                        .build();


                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequencenotAsync(dropPixelMiddle)),
                                new WaitCommand(1000),
                                new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequencenotAsync(backdropPixelMiddle)),
                                new WaitCommand(1000),
                                new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequencenotAsync(parkMiddle)),
                                new ParallelCommandGroup(
                                        new InstantCommand(() -> robot.a.armIntake()),
                                        new InstantCommand(() -> robot.angle.intake())
                                )
                        )
                );
                break;
            case RIGHT:
                TrajectorySequence dropPixelRight = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(18.89, -66.78, Math.toRadians(90.00)))
                        .splineToConstantHeading(new Vector2d(32.00, -42.75), Math.toRadians(90.00))
                        .build();

                TrajectorySequence backdropPixelRight = robot.driveSubsystem.trajectorySequenceBuilder(dropPixelRight.end())
                        .lineToConstantHeading(new Vector2d(37.35, -54.94))
                        .build();


                TrajectorySequence parkRight = robot.driveSubsystem.trajectorySequenceBuilder(backdropPixelRight.end())
                        .lineToSplineHeading(new Pose2d(71.5, -72.5, Math.toRadians(0.00)))
                        .build();

                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequencenotAsync(dropPixelRight)),
                                new WaitCommand(1000),
                                new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequencenotAsync(backdropPixelRight)),
                                new WaitCommand(1000),
                                new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequencenotAsync(parkRight)),
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
        colourMassDetectionProcessor.close();
        visionPortal.close();
        CommandScheduler.getInstance().reset();
    }
}