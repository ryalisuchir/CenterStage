package org.firstinspires.ftc.teamcode.Drive.OpModes.Worlds.Red.Right;

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
import org.firstinspires.ftc.teamcode.TrajectorySequences.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.DriveCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.LowOuttakeCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.LowerOuttakeCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.OuttakeCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.PlusTwoBlueStackCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.RestCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.SecondOuttakeCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.xPerimentalGrab;
import org.firstinspires.ftc.teamcode.Utility.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.RoadRunner.DriveConstants;
import org.firstinspires.ftc.teamcode.Utility.RoadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Utility.Vision.Prop.NewRedRightProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class RedRight extends OpMode {
    private RobotHardware robot;
    boolean previousX, currentX;
    boolean parkLeft = true, parkBoard = false, parkRight = false;
    boolean previousY, currentY;
    boolean plusTwo = false;

    private VisionPortal visionPortal;
    private NewRedRightProcessor colorMassDetectionProcessor;
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

        colorMassDetectionProcessor = new NewRedRightProcessor();
        colorMassDetectionProcessor.setDetectionColor(true); //false is blue, true is red
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .addProcessor(colorMassDetectionProcessor)
                .build();
    }

    public void init_loop() {
        currentX = gamepad1.x;
        currentY = gamepad1.y;

        if (currentX && !previousX) {
            if (parkLeft) {
                parkLeft = false;
                parkBoard = false;
                parkRight = true;
            } else if (parkRight) {
                parkLeft = false;
                parkBoard = true;
                parkRight = false;
            } else if (parkBoard) {
                parkLeft = true;
                parkBoard = false;
                parkRight = false;
            }
        }

        if (currentY && !previousY) {
            if (plusTwo) {
                plusTwo = false;
            } else if (!plusTwo) {
                plusTwo = true;
            }
        }


        telemetry.addData("Autonomous (Triangle): ", plusTwo ? "Plus Two" : "Regular 50");
        telemetry.addData("Parking On (Square): ", parkLeft ? "Left" : parkRight ? "Right" : parkBoard ? "At Board" : "");
        telemetry.addData("Currently Recorded Position: ", colorMassDetectionProcessor.getPropLocation());
        telemetry.addData("Camera State: ", visionPortal.getCameraState());
        telemetry.update();
        CommandScheduler.getInstance().run();
        robot.armSystem.loop();

        previousX = currentX;
        previousY = currentY;
    }

    @Override
    public void start() {
        time_since_start = new ElapsedTime();

        NewRedRightProcessor.PropPositions recordedPropPosition = colorMassDetectionProcessor.getPropLocation();
        if (plusTwo) {
            robot.driveSubsystem.setPoseEstimate(new Pose2d(18, -65.50, Math.toRadians(90.00)));
            if (parkLeft) {
                switch (recordedPropPosition) {
                    case MIDDLE:
                        TrajectorySequence movement1Middle = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(16.14, -63.32, Math.toRadians(90.00)))
                                .splineToConstantHeading(
                                        new Vector2d(21, -36), Math.toRadians(90.00),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(20.25, -50.07),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement2Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement1Middle.end())
                                .splineToLinearHeading(
                                        new Pose2d(55.5, -36, Math.toRadians(360.00)), Math.toRadians(360.00),
                                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement3Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement2Middle.end())
                                .setReversed(true)
                                .splineToLinearHeading(
                                        new Pose2d(26, -10.3, Math.toRadians(0)), Math.toRadians(180),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToLinearHeading(
                                        new Pose2d(-49, -10.3, Math.toRadians(0)), Math.toRadians(180),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToLinearHeading(
                                        new Pose2d(-55, -9.5, Math.toRadians(0)), Math.toRadians(180),
                                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement4Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement3Middle.end())
                                .splineToLinearHeading(new Pose2d(31, -10.3, Math.toRadians(0)), Math.toRadians(0))
                                .build();

                        TrajectorySequence movement5Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement4Middle.end())
                                .splineToConstantHeading(new Vector2d(48, -36.00), Math.toRadians(0))
                                .splineToLinearHeading(
                                        new Pose2d(53, -36.00, Math.toRadians(0.00)), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement6Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement5Middle.end())
                                .lineToConstantHeading(new Vector2d(40, -36.00))
                                .lineToConstantHeading(new Vector2d(40, -11))
                                .lineToConstantHeading(new Vector2d(57.00, -11))
                                .build();

                        CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement1Middle),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement2Middle),
                                                new LowerOuttakeCommand(robot)
                                        ),
                                        new WaitCommand(350),
                                        new InstantCommand(() -> robot.claw.releaseRight()),
                                        new WaitCommand(350),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement3Middle),
                                                new PlusTwoBlueStackCommand(robot)
                                        ),
                                        new xPerimentalGrab(robot),
                                        new WaitCommand(500),
                                        new DriveCommand(robot.driveSubsystem, movement4Middle),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement5Middle),
                                                new SecondOuttakeCommand(robot)
                                        ),
                                        new InstantCommand(() -> robot.claw.smallReleaseRight()),
                                        new WaitCommand(1000),
                                        new InstantCommand(() -> robot.claw.releaseRight()),
                                        new WaitCommand(350),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement6Middle),
                                                new RestCommand(robot)
                                        )
                                )
                        );
                        break;

                    case LEFT:
                    case UNFOUND:
                        TrajectorySequence movement1Left = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(16.14, -63.32, Math.toRadians(90.00)))
                                .splineToLinearHeading(
                                        new Pose2d(11, -40.5, Math.toRadians(120)), Math.toRadians(120),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement2Left = robot.driveSubsystem.trajectorySequenceBuilder(movement1Left.end())
                                .setReversed(true)
                                .splineToLinearHeading(
                                        new Pose2d(55, -30.5, Math.toRadians(0.00)), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement3Left = robot.driveSubsystem.trajectorySequenceBuilder(movement2Left.end())
                                .setReversed(true)
                                .splineToLinearHeading(
                                        new Pose2d(26, -10, Math.toRadians(0)), Math.toRadians(180),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToLinearHeading(
                                        new Pose2d(-49, -10, Math.toRadians(0)), Math.toRadians(180),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToLinearHeading(
                                        new Pose2d(-55.5, -9, Math.toRadians(0)), Math.toRadians(180),
                                        SampleMecanumDrive.getVelocityConstraint(9, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement4Left = robot.driveSubsystem.trajectorySequenceBuilder(movement3Left.end())
                                .splineToLinearHeading(new Pose2d(31, -10.5, Math.toRadians(0)), Math.toRadians(0))
                                .build();

                        TrajectorySequence movement5Left = robot.driveSubsystem.trajectorySequenceBuilder(movement4Left.end())
                                .splineToConstantHeading(new Vector2d(48, -36.00), Math.toRadians(0))
                                .splineToLinearHeading(
                                        new Pose2d(53, -36.00, Math.toRadians(0.00)), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement6Left = robot.driveSubsystem.trajectorySequenceBuilder(movement5Left.end())
                                .lineToConstantHeading(new Vector2d(40, -36.00))
                                .lineToConstantHeading(new Vector2d(40, -9))
                                .lineToConstantHeading(new Vector2d(57.00, -9))
                                .build();

                        CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement1Left),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement2Left),
                                                new OuttakeCommand(robot)
                                        ),
                                        new WaitCommand(350),
                                        new InstantCommand(() -> robot.claw.releaseRight()),
                                        new WaitCommand(350),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement3Left),
                                                new PlusTwoBlueStackCommand(robot)
                                        ),
                                        new xPerimentalGrab(robot),
                                        new WaitCommand(500),
                                        new DriveCommand(robot.driveSubsystem, movement4Left),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement5Left),
                                                new SecondOuttakeCommand(robot)
                                        ),
                                        new InstantCommand(() -> robot.claw.smallReleaseRight()),
                                        new WaitCommand(1000),
                                        new InstantCommand(() -> robot.claw.releaseRight()),
                                        new WaitCommand(350),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement6Left),
                                                new RestCommand(robot)
                                        )
                                )
                        );
                        break;
                    case RIGHT:
                        TrajectorySequence movement1Right = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(16.14, -63.32, Math.toRadians(90.00)))
                                .lineToConstantHeading(
                                        new Vector2d(29, -46),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(29, -56.52),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement2Right = robot.driveSubsystem.trajectorySequenceBuilder(movement1Right.end())
                                .splineToLinearHeading(
                                        new Pose2d(55, -43.5, Math.toRadians(0.00)), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement3Right = robot.driveSubsystem.trajectorySequenceBuilder(movement2Right.end())
                                .setReversed(true)
                                .splineToLinearHeading(
                                        new Pose2d(26, -8, Math.toRadians(0)), Math.toRadians(180),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToLinearHeading(
                                        new Pose2d(-49, -9, Math.toRadians(0)), Math.toRadians(180),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToLinearHeading(
                                        new Pose2d(-56, -9.5, Math.toRadians(0)), Math.toRadians(180),
                                        SampleMecanumDrive.getVelocityConstraint(9, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement4Right = robot.driveSubsystem.trajectorySequenceBuilder(movement3Right.end())
                                .splineToLinearHeading(new Pose2d(31, -9.5, Math.toRadians(0)), Math.toRadians(0))
                                .build();

                        TrajectorySequence movement5Right = robot.driveSubsystem.trajectorySequenceBuilder(movement4Right.end())
                                .splineToConstantHeading(new Vector2d(48, -36.00), Math.toRadians(0))
                                .splineToLinearHeading(
                                        new Pose2d(53, -36.00, Math.toRadians(0.00)), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement6Right = robot.driveSubsystem.trajectorySequenceBuilder(movement5Right.end())
                                .lineToConstantHeading(new Vector2d(40, -36.00))
                                .lineToConstantHeading(new Vector2d(40, -11))
                                .lineToConstantHeading(new Vector2d(57.00, -11))
                                .build();

                        CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement1Right),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement2Right),
                                                new OuttakeCommand(robot)
                                        ),
                                        new WaitCommand(350),
                                        new InstantCommand(() -> robot.claw.releaseRight()),
                                        new WaitCommand(350),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement3Right),
                                                new PlusTwoBlueStackCommand(robot)
                                        ),
                                        new xPerimentalGrab(robot),
                                        new WaitCommand(500),
                                        new DriveCommand(robot.driveSubsystem, movement4Right),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement5Right),
                                                new SecondOuttakeCommand(robot)
                                        ),
                                        new InstantCommand(() -> robot.claw.smallReleaseRight()),
                                        new WaitCommand(1000),
                                        new InstantCommand(() -> robot.claw.releaseRight()),
                                        new WaitCommand(350),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement6Right),
                                                new RestCommand(robot)
                                        )
                                )
                        );
                        break;
                }
            } else if (parkRight) {
                switch (recordedPropPosition) {
                    case MIDDLE:
                        TrajectorySequence movement1Middle = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(16.14, -63.32, Math.toRadians(90.00)))
                                .splineToConstantHeading(
                                        new Vector2d(21, -36), Math.toRadians(90.00),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(20.25, -50.07),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement2Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement1Middle.end())
                                .splineToLinearHeading(
                                        new Pose2d(55.5, -36, Math.toRadians(360.00)), Math.toRadians(360.00),
                                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement3Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement2Middle.end())
                                .setReversed(true)
                                .splineToLinearHeading(
                                        new Pose2d(26, -10.3, Math.toRadians(0)), Math.toRadians(180),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToLinearHeading(
                                        new Pose2d(-49, -9, Math.toRadians(0)), Math.toRadians(180),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToLinearHeading(
                                        new Pose2d(-55, -9, Math.toRadians(0)), Math.toRadians(180),
                                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement4Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement3Middle.end())
                                .splineToLinearHeading(new Pose2d(31, -10.3, Math.toRadians(0)), Math.toRadians(0))
                                .build();

                        TrajectorySequence movement5Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement4Middle.end())
                                .splineToConstantHeading(new Vector2d(48, -36.00), Math.toRadians(0))
                                .splineToLinearHeading(
                                        new Pose2d(53, -36.00, Math.toRadians(0.00)), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement6Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement5Middle.end())
                                .lineToConstantHeading(new Vector2d(40, -36.00))
                                .lineToConstantHeading(new Vector2d(40, -60))
                                .lineToConstantHeading(new Vector2d(57.00, -60))
                                .build();

                        CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement1Middle),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement2Middle),
                                                new LowerOuttakeCommand(robot)
                                        ),
                                        new WaitCommand(350),
                                        new InstantCommand(() -> robot.claw.releaseRight()),
                                        new WaitCommand(350),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement3Middle),
                                                new PlusTwoBlueStackCommand(robot)
                                        ),
                                        new xPerimentalGrab(robot),
                                        new WaitCommand(500),
                                        new DriveCommand(robot.driveSubsystem, movement4Middle),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement5Middle),
                                                new SecondOuttakeCommand(robot)
                                        ),
                                        new InstantCommand(() -> robot.claw.smallReleaseRight()),
                                        new WaitCommand(1000),
                                        new InstantCommand(() -> robot.claw.releaseRight()),
                                        new WaitCommand(350),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement6Middle),
                                                new RestCommand(robot)
                                        )
                                )
                        );
                        break;

                    case LEFT:
                    case UNFOUND:
                        TrajectorySequence movement1Left = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(16.14, -63.32, Math.toRadians(90.00)))
                                .splineToLinearHeading(
                                        new Pose2d(11, -40.5, Math.toRadians(120)), Math.toRadians(120),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement2Left = robot.driveSubsystem.trajectorySequenceBuilder(movement1Left.end())
                                .setReversed(true)
                                .splineToLinearHeading(
                                        new Pose2d(55, -30.5, Math.toRadians(0.00)), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement3Left = robot.driveSubsystem.trajectorySequenceBuilder(movement2Left.end())
                                .setReversed(true)
                                .splineToLinearHeading(
                                        new Pose2d(26, -10, Math.toRadians(0)), Math.toRadians(180),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToLinearHeading(
                                        new Pose2d(-49, -10, Math.toRadians(0)), Math.toRadians(180),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToLinearHeading(
                                        new Pose2d(-55.5, -9, Math.toRadians(0)), Math.toRadians(180),
                                        SampleMecanumDrive.getVelocityConstraint(9, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement4Left = robot.driveSubsystem.trajectorySequenceBuilder(movement3Left.end())
                                .splineToLinearHeading(new Pose2d(31, -10.5, Math.toRadians(0)), Math.toRadians(0))
                                .build();

                        TrajectorySequence movement5Left = robot.driveSubsystem.trajectorySequenceBuilder(movement4Left.end())
                                .splineToConstantHeading(new Vector2d(48, -36.00), Math.toRadians(0))
                                .splineToLinearHeading(
                                        new Pose2d(53, -36.00, Math.toRadians(0.00)), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement6Left = robot.driveSubsystem.trajectorySequenceBuilder(movement5Left.end())
                                .lineToConstantHeading(new Vector2d(40, -36.00))
                                .lineToConstantHeading(new Vector2d(40, -60))
                                .lineToConstantHeading(new Vector2d(57.00, -60))
                                .build();

                        CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement1Left),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement2Left),
                                                new OuttakeCommand(robot)
                                        ),
                                        new WaitCommand(350),
                                        new InstantCommand(() -> robot.claw.releaseRight()),
                                        new WaitCommand(350),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement3Left),
                                                new PlusTwoBlueStackCommand(robot)
                                        ),
                                        new xPerimentalGrab(robot),
                                        new WaitCommand(500),
                                        new DriveCommand(robot.driveSubsystem, movement4Left),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement5Left),
                                                new SecondOuttakeCommand(robot)
                                        ),
                                        new InstantCommand(() -> robot.claw.smallReleaseRight()),
                                        new WaitCommand(1000),
                                        new InstantCommand(() -> robot.claw.releaseRight()),
                                        new WaitCommand(350),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement6Left),
                                                new RestCommand(robot)
                                        )
                                )
                        );
                        break;
                    case RIGHT:
                        TrajectorySequence movement1Right = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(16.14, -63.32, Math.toRadians(90.00)))
                                .lineToConstantHeading(
                                        new Vector2d(29, -46),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(29, -56.52),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement2Right = robot.driveSubsystem.trajectorySequenceBuilder(movement1Right.end())
                                .splineToLinearHeading(
                                        new Pose2d(55, -43.5, Math.toRadians(0.00)), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement3Right = robot.driveSubsystem.trajectorySequenceBuilder(movement2Right.end())
                                .setReversed(true)
                                .splineToLinearHeading(
                                        new Pose2d(26, -8, Math.toRadians(0)), Math.toRadians(180),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToLinearHeading(
                                        new Pose2d(-49, -9, Math.toRadians(0)), Math.toRadians(180),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToLinearHeading(
                                        new Pose2d(-56, -9.5, Math.toRadians(0)), Math.toRadians(180),
                                        SampleMecanumDrive.getVelocityConstraint(9, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement4Right = robot.driveSubsystem.trajectorySequenceBuilder(movement3Right.end())
                                .splineToLinearHeading(new Pose2d(31, -9.5, Math.toRadians(0)), Math.toRadians(0))
                                .build();

                        TrajectorySequence movement5Right = robot.driveSubsystem.trajectorySequenceBuilder(movement4Right.end())
                                .splineToConstantHeading(new Vector2d(48, -36.00), Math.toRadians(0))
                                .splineToLinearHeading(
                                        new Pose2d(53, -36.00, Math.toRadians(0.00)), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement6Right = robot.driveSubsystem.trajectorySequenceBuilder(movement5Right.end())
                                .lineToConstantHeading(new Vector2d(40, -36.00))
                                .lineToConstantHeading(new Vector2d(40, -60))
                                .lineToConstantHeading(new Vector2d(57.00, -60))
                                .build();

                        CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement1Right),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement2Right),
                                                new OuttakeCommand(robot)
                                        ),
                                        new WaitCommand(350),
                                        new InstantCommand(() -> robot.claw.releaseRight()),
                                        new WaitCommand(350),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement3Right),
                                                new PlusTwoBlueStackCommand(robot)
                                        ),
                                        new xPerimentalGrab(robot),
                                        new WaitCommand(500),
                                        new DriveCommand(robot.driveSubsystem, movement4Right),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement5Right),
                                                new SecondOuttakeCommand(robot)
                                        ),
                                        new InstantCommand(() -> robot.claw.smallReleaseRight()),
                                        new WaitCommand(1000),
                                        new InstantCommand(() -> robot.claw.releaseRight()),
                                        new WaitCommand(350),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement6Right),
                                                new RestCommand(robot)
                                        )
                                )
                        );
                        break;
                }
            } else if (parkBoard) {
                switch (recordedPropPosition) {
                    case MIDDLE:
                        TrajectorySequence movement1Middle = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(16.14, -63.32, Math.toRadians(90.00)))
                                .splineToConstantHeading(
                                        new Vector2d(21, -36), Math.toRadians(90.00),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(20.25, -50.07),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement2Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement1Middle.end())
                                .splineToLinearHeading(
                                        new Pose2d(55.5, -36, Math.toRadians(360.00)), Math.toRadians(360.00),
                                        SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement3Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement2Middle.end())
                                .setReversed(true)
                                .splineToLinearHeading(
                                        new Pose2d(26, -10.3, Math.toRadians(0)), Math.toRadians(180),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToLinearHeading(
                                        new Pose2d(-49, -9, Math.toRadians(0)), Math.toRadians(180),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToLinearHeading(
                                        new Pose2d(-55, -8.5, Math.toRadians(0)), Math.toRadians(180),
                                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement4Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement3Middle.end())
                                .splineToLinearHeading(new Pose2d(31, -10.3, Math.toRadians(0)), Math.toRadians(0))
                                .build();

                        TrajectorySequence movement5Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement4Middle.end())
                                .splineToConstantHeading(new Vector2d(48, -36.00), Math.toRadians(0))
                                .splineToLinearHeading(
                                        new Pose2d(53, -36.00, Math.toRadians(0.00)), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement6Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement5Middle.end())
                                .back(6)
                                .build();

                        CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement1Middle),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement2Middle),
                                                new LowerOuttakeCommand(robot)
                                        ),
                                        new WaitCommand(350),
                                        new InstantCommand(() -> robot.claw.releaseRight()),
                                        new WaitCommand(350),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement3Middle),
                                                new PlusTwoBlueStackCommand(robot)
                                        ),
                                        new xPerimentalGrab(robot),
                                        new WaitCommand(500),
                                        new DriveCommand(robot.driveSubsystem, movement4Middle),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement5Middle),
                                                new SecondOuttakeCommand(robot)
                                        ),
                                        new InstantCommand(() -> robot.claw.smallReleaseRight()),
                                        new WaitCommand(1000),
                                        new InstantCommand(() -> robot.claw.releaseRight()),
                                        new WaitCommand(350),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement6Middle),
                                                new RestCommand(robot)
                                        )
                                )
                        );
                        break;

                    case LEFT:
                    case UNFOUND:
                        TrajectorySequence movement1Left = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(16.14, -63.32, Math.toRadians(90.00)))
                                .splineToLinearHeading(
                                        new Pose2d(11, -40.5, Math.toRadians(120)), Math.toRadians(120),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement2Left = robot.driveSubsystem.trajectorySequenceBuilder(movement1Left.end())
                                .setReversed(true)
                                .splineToLinearHeading(
                                        new Pose2d(55, -30.5, Math.toRadians(0.00)), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement3Left = robot.driveSubsystem.trajectorySequenceBuilder(movement2Left.end())
                                .setReversed(true)
                                .splineToLinearHeading(
                                        new Pose2d(26, -10, Math.toRadians(0)), Math.toRadians(180),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToLinearHeading(
                                        new Pose2d(-49, -10, Math.toRadians(0)), Math.toRadians(180),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToLinearHeading(
                                        new Pose2d(-55.5, -9.5, Math.toRadians(0)), Math.toRadians(180),
                                        SampleMecanumDrive.getVelocityConstraint(9, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement4Left = robot.driveSubsystem.trajectorySequenceBuilder(movement3Left.end())
                                .splineToLinearHeading(new Pose2d(31, -10.5, Math.toRadians(0)), Math.toRadians(0))
                                .build();

                        TrajectorySequence movement5Left = robot.driveSubsystem.trajectorySequenceBuilder(movement4Left.end())
                                .splineToConstantHeading(new Vector2d(48, -36.00), Math.toRadians(0))
                                .splineToLinearHeading(
                                        new Pose2d(53, -36.00, Math.toRadians(0.00)), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement6Left = robot.driveSubsystem.trajectorySequenceBuilder(movement5Left.end())
                                .back(6)
                                .build();

                        CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement1Left),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement2Left),
                                                new OuttakeCommand(robot)
                                        ),
                                        new WaitCommand(350),
                                        new InstantCommand(() -> robot.claw.releaseRight()),
                                        new WaitCommand(350),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement3Left),
                                                new PlusTwoBlueStackCommand(robot)
                                        ),
                                        new xPerimentalGrab(robot),
                                        new WaitCommand(500),
                                        new DriveCommand(robot.driveSubsystem, movement4Left),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement5Left),
                                                new SecondOuttakeCommand(robot)
                                        ),
                                        new InstantCommand(() -> robot.claw.smallReleaseRight()),
                                        new WaitCommand(1000),
                                        new InstantCommand(() -> robot.claw.releaseRight()),
                                        new WaitCommand(350),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement6Left),
                                                new RestCommand(robot)
                                        )
                                )
                        );
                        break;
                    case RIGHT:
                        TrajectorySequence movement1Right = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(16.14, -63.32, Math.toRadians(90.00)))
                                .lineToConstantHeading(
                                        new Vector2d(29, -46),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(29, -56.52),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement2Right = robot.driveSubsystem.trajectorySequenceBuilder(movement1Right.end())
                                .splineToLinearHeading(
                                        new Pose2d(55, -43.5, Math.toRadians(0.00)), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement3Right = robot.driveSubsystem.trajectorySequenceBuilder(movement2Right.end())
                                .setReversed(true)
                                .splineToLinearHeading(
                                        new Pose2d(26, -8, Math.toRadians(0)), Math.toRadians(180),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToLinearHeading(
                                        new Pose2d(-49, -9, Math.toRadians(0)), Math.toRadians(180),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToLinearHeading(
                                        new Pose2d(-56, -9.5, Math.toRadians(0)), Math.toRadians(180),
                                        SampleMecanumDrive.getVelocityConstraint(9, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement4Right = robot.driveSubsystem.trajectorySequenceBuilder(movement3Right.end())
                                .splineToLinearHeading(new Pose2d(31, -9.5, Math.toRadians(0)), Math.toRadians(0))
                                .build();

                        TrajectorySequence movement5Right = robot.driveSubsystem.trajectorySequenceBuilder(movement4Right.end())
                                .splineToConstantHeading(new Vector2d(48, -36.00), Math.toRadians(0))
                                .splineToLinearHeading(
                                        new Pose2d(53, -36.00, Math.toRadians(0.00)), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement6Right = robot.driveSubsystem.trajectorySequenceBuilder(movement5Right.end())
                                .back(6)
                                .build();

                        CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement1Right),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement2Right),
                                                new OuttakeCommand(robot)
                                        ),
                                        new WaitCommand(350),
                                        new InstantCommand(() -> robot.claw.releaseRight()),
                                        new WaitCommand(350),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement3Right),
                                                new PlusTwoBlueStackCommand(robot)
                                        ),
                                        new xPerimentalGrab(robot),
                                        new WaitCommand(500),
                                        new DriveCommand(robot.driveSubsystem, movement4Right),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement5Right),
                                                new SecondOuttakeCommand(robot)
                                        ),
                                        new InstantCommand(() -> robot.claw.smallReleaseRight()),
                                        new WaitCommand(1000),
                                        new InstantCommand(() -> robot.claw.releaseRight()),
                                        new WaitCommand(350),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement6Right),
                                                new RestCommand(robot)
                                        )
                                )
                        );
                        break;
                }
            }
        } else if (!plusTwo) {
            robot.driveSubsystem.setPoseEstimate(new Pose2d(16.14, -63.32, Math.toRadians(90.00)));
            if (parkLeft) {
                switch (recordedPropPosition) {
                    case LEFT:
                    case UNFOUND:
                        TrajectorySequence movement1Left = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(16.14, -63.32, Math.toRadians(90.00)))
                                .splineToLinearHeading(
                                        new Pose2d(9, -40.5, Math.toRadians(120)), Math.toRadians(120),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement2Left = robot.driveSubsystem.trajectorySequenceBuilder(movement1Left.end())
                                .setReversed(true)
                                .splineToLinearHeading(
                                        new Pose2d(53.5, -29, Math.toRadians(0.00)), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();


                        TrajectorySequence movement3Left = robot.driveSubsystem.trajectorySequenceBuilder(movement2Left.end())
                                .lineToConstantHeading(
                                        new Vector2d(40, -29.5),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(40, -9),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(57, -9),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement1Left),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement2Left),
                                                new OuttakeCommand(robot)
                                        ),
                                        new WaitCommand(750),
                                        new InstantCommand(() -> robot.claw.releaseRight()),
                                        new WaitCommand(750),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement3Left),
                                                new RestCommand(robot)
                                        )

                                )
                        );
                        break;
                    case RIGHT:
                        TrajectorySequence movement1Right = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(16.14, -63.32, Math.toRadians(90.00)))
                                .splineToConstantHeading(
                                        new Vector2d(27.80, -43.31), Math.toRadians(90.00),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(27.80, -56.52),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement2Right = robot.driveSubsystem.trajectorySequenceBuilder(movement1Right.end())
                                .splineToLinearHeading(
                                        new Pose2d(54.2, -40, Math.toRadians(0.00)), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement3Right = robot.driveSubsystem.trajectorySequenceBuilder(movement2Right.end())
                                .lineToConstantHeading(
                                        new Vector2d(38.41, -39),
                                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(39.06, -9.5),
                                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(58, -9.5),
                                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement1Right),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement2Right),
                                                new OuttakeCommand(robot)
                                        ),
                                        new WaitCommand(750),
                                        new InstantCommand(() -> robot.claw.releaseRight()),
                                        new WaitCommand(750),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement3Right),
                                                new RestCommand(robot)
                                        )

                                )
                        );
                        break;
                    case MIDDLE:
                        TrajectorySequence movement1Middle = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(16.14, -63.32, Math.toRadians(450.00)))
                                .splineToConstantHeading(
                                        new Vector2d(20.25, -33.88), Math.toRadians(90.00),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(20.25, -50.07),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement2Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement1Middle.end())
                                .splineToLinearHeading(
                                        new Pose2d(54.2, -35, Math.toRadians(360.00)), Math.toRadians(360.00),
                                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement3Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement2Middle.end())
                                .lineToConstantHeading(
                                        new Vector2d(38.41, -31),
                                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(39.06, -61),
                                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(61.01, -61),
                                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement1Middle),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement2Middle),
                                                new OuttakeCommand(robot)
                                        ),
                                        new WaitCommand(750),
                                        new InstantCommand(() -> robot.claw.releaseRight()),
                                        new WaitCommand(750),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement3Middle),
                                                new RestCommand(robot)
                                        )

                                )
                        );
                        break;
                }
            } else if (parkRight) {
                switch (recordedPropPosition) {
                    case LEFT:
                    case UNFOUND:
                        TrajectorySequence movement1Left = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(16.14, -63.32, Math.toRadians(90.00)))
                                .splineToLinearHeading(
                                        new Pose2d(9, -40.5, Math.toRadians(120)), Math.toRadians(120),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement2Left = robot.driveSubsystem.trajectorySequenceBuilder(movement1Left.end())
                                .setReversed(true)
                                .splineToLinearHeading(
                                        new Pose2d(53.5, -29, Math.toRadians(0.00)), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();


                        TrajectorySequence movement3Left = robot.driveSubsystem.trajectorySequenceBuilder(movement2Left.end())
                                .lineToConstantHeading(
                                        new Vector2d(40, -29.5),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(40, -61),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(57, -61),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement1Left),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement2Left),
                                                new OuttakeCommand(robot)
                                        ),
                                        new WaitCommand(750),
                                        new InstantCommand(() -> robot.claw.releaseRight()),
                                        new WaitCommand(750),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement3Left),
                                                new RestCommand(robot)
                                        )

                                )
                        );
                        break;
                    case RIGHT:
                        TrajectorySequence movement1Right = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(16.14, -63.32, Math.toRadians(90.00)))
                                .splineToConstantHeading(
                                        new Vector2d(27.80, -43.31), Math.toRadians(90.00),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(27.80, -56.52),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement2Right = robot.driveSubsystem.trajectorySequenceBuilder(movement1Right.end())
                                .splineToLinearHeading(
                                        new Pose2d(54.2, -40, Math.toRadians(0.00)), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement3Right = robot.driveSubsystem.trajectorySequenceBuilder(movement2Right.end())
                                .lineToConstantHeading(
                                        new Vector2d(38.41, -39),
                                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(39.06, -61),
                                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(58, -61),
                                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement1Right),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement2Right),
                                                new OuttakeCommand(robot)
                                        ),
                                        new WaitCommand(750),
                                        new InstantCommand(() -> robot.claw.releaseRight()),
                                        new WaitCommand(750),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement3Right),
                                                new RestCommand(robot)
                                        )

                                )
                        );
                        break;
                    case MIDDLE:
                        TrajectorySequence movement1Middle = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(16.14, -63.32, Math.toRadians(450.00)))
                                .splineToConstantHeading(
                                        new Vector2d(20.25, -33.88), Math.toRadians(90.00),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(20.25, -50.07),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement2Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement1Middle.end())
                                .splineToLinearHeading(
                                        new Pose2d(54.2, -35, Math.toRadians(360.00)), Math.toRadians(360.00),
                                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement3Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement2Middle.end())
                                .lineToConstantHeading(
                                        new Vector2d(38.41, -31),
                                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(39.06, -9.5),
                                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(61.01, -9.5),
                                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement1Middle),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement2Middle),
                                                new OuttakeCommand(robot)
                                        ),
                                        new WaitCommand(750),
                                        new InstantCommand(() -> robot.claw.releaseRight()),
                                        new WaitCommand(750),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement3Middle),
                                                new RestCommand(robot)
                                        )

                                )
                        );
                        break;
                }

            } else if (parkBoard) {
                switch (recordedPropPosition) {
                    case LEFT:
                    case UNFOUND:
                        TrajectorySequence movement1Left = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(16.14, -63.32, Math.toRadians(90.00)))
                                .splineToLinearHeading(
                                        new Pose2d(9, -40.5, Math.toRadians(120)), Math.toRadians(120),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement2Left = robot.driveSubsystem.trajectorySequenceBuilder(movement1Left.end())
                                .setReversed(true)
                                .splineToLinearHeading(
                                        new Pose2d(53.5, -29, Math.toRadians(0.00)), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();


                        TrajectorySequence movement3Left = robot.driveSubsystem.trajectorySequenceBuilder(movement2Left.end())
                                .back(6)
                                .build();

                        CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement1Left),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement2Left),
                                                new OuttakeCommand(robot)
                                        ),
                                        new WaitCommand(750),
                                        new InstantCommand(() -> robot.claw.releaseRight()),
                                        new WaitCommand(750),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement3Left),
                                                new RestCommand(robot)
                                        )

                                )
                        );
                        break;
                    case RIGHT:
                        TrajectorySequence movement1Right = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(16.14, -63.32, Math.toRadians(90.00)))
                                .splineToConstantHeading(
                                        new Vector2d(27.80, -43.31), Math.toRadians(90.00),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(27.80, -56.52),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement2Right = robot.driveSubsystem.trajectorySequenceBuilder(movement1Right.end())
                                .splineToLinearHeading(
                                        new Pose2d(54.2, -40, Math.toRadians(0.00)), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement3Right = robot.driveSubsystem.trajectorySequenceBuilder(movement2Right.end())
                                .back(6)
                                .build();

                        CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement1Right),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement2Right),
                                                new OuttakeCommand(robot)
                                        ),
                                        new WaitCommand(750),
                                        new InstantCommand(() -> robot.claw.releaseRight()),
                                        new WaitCommand(750),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement3Right),
                                                new RestCommand(robot)
                                        )

                                )
                        );
                        break;
                    case MIDDLE:
                        TrajectorySequence movement1Middle = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(16.14, -63.32, Math.toRadians(450.00)))
                                .splineToConstantHeading(
                                        new Vector2d(20.25, -33.88), Math.toRadians(90.00),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(20.25, -50.07),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement2Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement1Middle.end())
                                .splineToLinearHeading(
                                        new Pose2d(54.2, -35, Math.toRadians(360.00)), Math.toRadians(360.00),
                                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement3Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement2Middle.end())
                                .back(6)
                                .build();

                        CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement1Middle),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement2Middle),
                                                new OuttakeCommand(robot)
                                        ),
                                        new WaitCommand(750),
                                        new InstantCommand(() -> robot.claw.releaseRight()),
                                        new WaitCommand(750),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement3Middle),
                                                new RestCommand(robot)
                                        )

                                )
                        );
                        break;
                }
            }
        }
    }
    @Override
    public void loop() {

        CommandScheduler.getInstance().run();
        robot.armSystem.loop();
        robot.slidesSubsystem.loop();
        robot.driveSubsystem.update();

        double time = System.currentTimeMillis();
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
