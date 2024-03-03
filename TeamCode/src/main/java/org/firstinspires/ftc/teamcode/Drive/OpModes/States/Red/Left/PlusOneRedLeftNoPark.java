package org.firstinspires.ftc.teamcode.Drive.OpModes.States.Red.Left;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.Drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.TrajectorySequences.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.DriveCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.LowOuttakeCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.NewRedStackCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.RedStackCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.RestCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.SecondOuttakeCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.SlidesDownCommand;
import org.firstinspires.ftc.teamcode.Utility.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.Vision.Prop.NewRedLeftProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class PlusOneRedLeftNoPark extends OpMode {
    private VisionPortal visionPortal;
    private NewRedLeftProcessor colorMassDetectionProcessor;

    private RobotHardware robot;
    private ElapsedTime time_since_start;
    private double loop;

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

        colorMassDetectionProcessor = new NewRedLeftProcessor();
        colorMassDetectionProcessor.setDetectionColor(true); //false is blue, true is red
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .addProcessor(colorMassDetectionProcessor)
                .build();
    }

    @Override
    public void init_loop() {
        telemetry.addData("Currently Recorded Position: ", colorMassDetectionProcessor.getPropLocation());
        telemetry.addData("Camera State: ", visionPortal.getCameraState());
        CommandScheduler.getInstance().run();
        robot.armSystem.loop();
    }

    @Override
    public void start() {
        time_since_start = new ElapsedTime();
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.stopLiveView();
            visionPortal.stopStreaming();
        }

        NewRedLeftProcessor.PropPositions recordedPropPosition = colorMassDetectionProcessor.getPropLocation();
        robot.driveSubsystem.setPoseEstimate(new Pose2d(-40.11, -63.48, Math.toRadians(450.00)));

        switch (recordedPropPosition) {
            case RIGHT:
                TrajectorySequence movement1Right = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, -63.48, Math.toRadians(90.00)))
                        .splineTo(
                                new Vector2d(-34.99, -37.93), Math.toRadians(45.00),
                                SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToSplineHeading(
                                new Pose2d(-55, -41, Math.toRadians(360.00)),
                                SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToSplineHeading(
                                new Pose2d(-55, -37.5, Math.toRadians(360.00)),
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build(); //drop arm to intake open claw

                TrajectorySequence lilMoreCuh = robot.driveSubsystem.trajectorySequenceBuilder(movement1Right.end())
                        .lineToConstantHeading(
                                new Vector2d(-58.5, -37.5),
                                SampleMecanumDrive.getVelocityConstraint(7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();


                TrajectorySequence movement2Right = robot.driveSubsystem.trajectorySequenceBuilder(lilMoreCuh.end())
                        .lineToConstantHeading(new Vector2d(-56.34, -38.4))
                        .lineToConstantHeading(new Vector2d(-57.47, -58.5))
                        .lineToConstantHeading(new Vector2d(20, -57))
                        .build();


                TrajectorySequence movement3Right = robot.driveSubsystem.trajectorySequenceBuilder(movement2Right.end())
                        .splineToConstantHeading(
                                new Vector2d(50, -35), Math.toRadians(0.00),
                                SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();


                TrajectorySequence movement4Right = robot.driveSubsystem.trajectorySequenceBuilder(movement3Right.end())
//                        .lineToConstantHeading(
//                                new Vector2d(48, 37),
//                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
//                        )
                        .setTangent(-180)
                        .splineToConstantHeading(
                                new Vector2d(
                                        48.5, -30), Math.toRadians(0),
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence movement5Right = robot.driveSubsystem.trajectorySequenceBuilder(movement4Right.end())
                        .back(5)
                        .build();

                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new DriveCommand(robot.driveSubsystem, movement1Right),
                                new NewRedStackCommand(robot),
                                new WaitCommand(500),
                                new DriveCommand(robot.driveSubsystem, lilMoreCuh),
                                new WaitCommand(500),
                                new InstantCommand(() -> robot.claw.grabBoth()),
                                new DriveCommand(robot.driveSubsystem, movement2Right),
                                new ParallelCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement3Right),
                                        new LowOuttakeCommand(robot)
                                ),
                                new WaitCommand(350),
                                new InstantCommand(() -> robot.claw.releaseRight()),
                                new WaitCommand(350),
                                new ParallelCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement4Right),
                                        new SecondOuttakeCommand(robot)
                                ),
                                new WaitCommand(600),
                                new InstantCommand(() -> robot.claw.releaseLeft()),
                                new WaitCommand(1500),
                                new ParallelCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement5Right),
                                        new RestCommand(robot)
                                )

                        )
                );

                break;
            case LEFT:
            case UNFOUND:
                TrajectorySequence movement1Left = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, -63.48, Math.toRadians(90.00)))
                        .splineToConstantHeading(
                                new Vector2d(-50.00, -43.70), Math.toRadians(450.00),
                                SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToSplineHeading(
                                new Pose2d(-55.5, -55.39, Math.toRadians(360.00)),
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToSplineHeading(
                                new Pose2d(-55.5, -38, Math.toRadians(360.00)),
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build(); //drop arm to intake open claw

                TrajectorySequence lilMoreCuh2 = robot.driveSubsystem.trajectorySequenceBuilder(movement1Left.end())
                        .back(
                                3,
                                SampleMecanumDrive.getVelocityConstraint(7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence movement2Left = robot.driveSubsystem.trajectorySequenceBuilder(lilMoreCuh2.end())
                        .lineToConstantHeading(new Vector2d(-56.34, -38.4))
                        .lineToConstantHeading(new Vector2d(-57.47, -58.5))
                        .lineToConstantHeading(new Vector2d(20, -57))
                        .build();

                TrajectorySequence movement3Left = robot.driveSubsystem.trajectorySequenceBuilder(movement2Left.end())
                        .splineToConstantHeading(
                                new Vector2d(49.3, -23), Math.toRadians(0.00),
                                SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence movement4Left = robot.driveSubsystem.trajectorySequenceBuilder(movement3Left.end())
//                        .lineToConstantHeading(
//                                new Vector2d(48, 37),
//                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
//                        )
                        .setTangent(-180)
                        .splineToConstantHeading(
                                new Vector2d(
                                        48, -37), Math.toRadians(0),
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence movement5Left = robot.driveSubsystem.trajectorySequenceBuilder(movement4Left.end())
                        .back(5)
                        .build();

                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new DriveCommand(robot.driveSubsystem, movement1Left),
                                new NewRedStackCommand(robot),
                                new WaitCommand(500),
                                new DriveCommand(robot.driveSubsystem, lilMoreCuh2),
                                new WaitCommand(500),
                                new InstantCommand(() -> robot.claw.grabBoth()),
                                new DriveCommand(robot.driveSubsystem, movement2Left),
                                new ParallelCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement3Left),
                                        new LowOuttakeCommand(robot)
                                ),
                                new WaitCommand(350),
                                new InstantCommand(() -> robot.claw.releaseRight()),
                                new WaitCommand(350),
                                new ParallelCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement4Left),
                                        new SecondOuttakeCommand(robot)
                                ),
                                new WaitCommand(600),
                                new InstantCommand(() -> robot.claw.releaseLeft()),
                                new WaitCommand(1500),
                                new ParallelCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement5Left),
                                        new RestCommand(robot)
                                )

                        )
                );
                break;
            case MIDDLE:
                TrajectorySequence movement1Middle = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, -63.48, Math.toRadians(90.00)))
                        .splineToConstantHeading(
                                new Vector2d(-44.50, -33.10), Math.toRadians(90.00),
                                SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToSplineHeading(
                                new Pose2d(-55, -41, Math.toRadians(360.00)),
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToSplineHeading(
                                new Pose2d(-55, -37.5, Math.toRadians(360.00)),
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build(); //drop arm to intake open claw


                TrajectorySequence lilMoreCuh3 = robot.driveSubsystem.trajectorySequenceBuilder(movement1Middle.end())
                        .back(
                                3.5,
                                SampleMecanumDrive.getVelocityConstraint(7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();


                TrajectorySequence movement2Middle = robot.driveSubsystem.trajectorySequenceBuilder(lilMoreCuh3.end())
                        .lineToConstantHeading(new Vector2d(-56.34, -38.4))
                        .lineToConstantHeading(new Vector2d(-57.47, -58.5))
                        .lineToConstantHeading(new Vector2d(20, -57))
                        .build();


                TrajectorySequence movement3Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement2Middle.end())
                        .splineToConstantHeading(
                                new Vector2d(52, -30.5), Math.toRadians(0.00),
                                SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();


                TrajectorySequence movement4Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement3Middle.end())
//                        .lineToConstantHeading(
//                                new Vector2d(48, 37),
//                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
//                        )
                        .setTangent(-180)
                        .splineToConstantHeading(
                                new Vector2d(
                                        48.9, -30), Math.toRadians(0),
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();


                TrajectorySequence movement5Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement4Middle.end())
                        .back(5)
                        .build();


                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new DriveCommand(robot.driveSubsystem, movement1Middle),
                                new NewRedStackCommand(robot),
                                new WaitCommand(500),
                                new DriveCommand(robot.driveSubsystem, lilMoreCuh3),
                                new WaitCommand(500),
                                new InstantCommand(() -> robot.claw.grabBoth()),
                                new DriveCommand(robot.driveSubsystem, movement2Middle),
                                new ParallelCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement3Middle),
                                        new LowOuttakeCommand(robot)
                                ),
                                new WaitCommand(350),
                                new InstantCommand(() -> robot.claw.releaseRight()),
                                new WaitCommand(350),
                                new ParallelCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement4Middle),
                                        new SecondOuttakeCommand(robot)
                                ),
                                new WaitCommand(600),
                                new InstantCommand(() -> robot.claw.releaseLeft()),
                                new WaitCommand(1500),
                                new ParallelCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement5Middle),
                                        new RestCommand(robot)
                                )


                        )
                );
                break;
        }
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        robot.armSystem.loop();
        robot.slidesSubsystem.loop();
        robot.driveSubsystem.update();

        double time = System.currentTimeMillis();
        telemetry.addData("loop ", time - loop);
        telemetry.addData("Time Elapsed: ", time_since_start);
        telemetry.addData("Current Loop Time: ", time - loop);

        loop = time;

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