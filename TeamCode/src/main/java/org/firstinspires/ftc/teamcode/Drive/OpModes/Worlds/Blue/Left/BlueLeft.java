package org.firstinspires.ftc.teamcode.Drive.OpModes.Worlds.Blue.Left;

import com.acmerobotics.dashboard.FtcDashboard;
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
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.LowerOuttakeCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.OuttakeCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.PlusTwoBlueStackCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.RestCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.SecondOuttakeCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.xPerimentalGrab;
import org.firstinspires.ftc.teamcode.Utility.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.RoadRunner.DriveConstants;
import org.firstinspires.ftc.teamcode.Utility.RoadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Utility.Vision.Prop.NewBlueLeftProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class BlueLeft extends OpMode {
    private RobotHardware robot;
    boolean previousX, currentX;
    boolean parkLeft = true, parkBoard = false, parkRight = false;
    boolean previousY, currentY;
    boolean plusTwo = false;

    private VisionPortal visionPortal;
    private NewBlueLeftProcessor colorMassDetectionProcessor;

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

        colorMassDetectionProcessor = new NewBlueLeftProcessor();
        colorMassDetectionProcessor.setDetectionColor(false); //false is blue, true is red

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .addProcessor(colorMassDetectionProcessor)
                .build();

        FtcDashboard.getInstance().startCameraStream(colorMassDetectionProcessor, 30);
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

        NewBlueLeftProcessor.PropPositions recordedPropPosition = colorMassDetectionProcessor.getPropLocation();
        robot.driveSubsystem.setPoseEstimate(new Pose2d(18, 65.50, Math.toRadians(270.00)));

        if (plusTwo) { //PLUS TWO
            if (parkLeft) {
                switch (recordedPropPosition) {
                    case MIDDLE:
                        TrajectorySequence movement1Middle = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(18, 65.50, Math.toRadians(270.00)))
                                .splineToConstantHeading(
                                        new Vector2d(21, 37), Math.toRadians(270.00),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(27.52, 53),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement2Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement1Middle.end())
                                .splineToSplineHeading(
                                        new Pose2d(52.3, 36, Math.toRadians(0.00)), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement3Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement2Middle.end())
                                .setReversed(true)
                                .splineToConstantHeading(
                                        new Vector2d(20, 15), Math.toRadians(180.00),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToConstantHeading(
                                        new Vector2d(-42, 15), Math.toRadians(180.00),
                                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToConstantHeading(
                                        new Vector2d(-58.5, 14.5), Math.toRadians(180.00),
                                        SampleMecanumDrive.getVelocityConstraint(8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .setReversed(false)
                                .build();

                        TrajectorySequence movement4Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement3Middle.end())
                                .splineToConstantHeading(
                                        new Vector2d(28.98, 14), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement5Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement4Middle.end())
                                .splineToConstantHeading(
                                        new Vector2d(45, 38), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToConstantHeading(
                                        new Vector2d(50, 38), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement6Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement5Middle.end())
                                .lineToConstantHeading(
                                        new Vector2d(38, 38),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(38, 69),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(57.00, 69),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement1Middle),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement2Middle),
                                                new LowerOuttakeCommand(robot)
                                        ),
                                        new WaitCommand(350),
                                        new InstantCommand(() -> robot.claw.releaseLeft()),
                                        new WaitCommand(350),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement3Middle),
                                                new PlusTwoBlueStackCommand(robot)
                                        ),
                                        //                                new InstantCommand(() -> robot.claw.grabBoth()),
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
                        TrajectorySequence movement1Left = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(18, 65.50, Math.toRadians(270.00)))
                                .splineToConstantHeading(
                                        new Vector2d(27.8, 43.31), Math.toRadians(270.00),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(27.52, 53),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement2Left = robot.driveSubsystem.trajectorySequenceBuilder(movement1Left.end())
                                .splineToSplineHeading(
                                        new Pose2d(52.3, 41, Math.toRadians(0.00)), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement3Left = robot.driveSubsystem.trajectorySequenceBuilder(movement2Left.end())
                                .setReversed(true)
                                .splineToConstantHeading(
                                        new Vector2d(20, 15), Math.toRadians(180.00),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToConstantHeading(
                                        new Vector2d(-42, 15), Math.toRadians(180.00),
                                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToConstantHeading(
                                        new Vector2d(-58.5, 14.5), Math.toRadians(180.00),
                                        SampleMecanumDrive.getVelocityConstraint(8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .setReversed(false)
                                .build();

                        TrajectorySequence movement4Left = robot.driveSubsystem.trajectorySequenceBuilder(movement3Left.end())
                                .splineToConstantHeading(
                                        new Vector2d(28.98, 14), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement5Left = robot.driveSubsystem.trajectorySequenceBuilder(movement4Left.end())
                                .splineToConstantHeading(
                                        new Vector2d(45, 38), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToConstantHeading(
                                        new Vector2d(50.5, 38), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement6Left = robot.driveSubsystem.trajectorySequenceBuilder(movement5Left.end())
                                .lineToConstantHeading(
                                        new Vector2d(38, 38),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(38, 69),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(57.00, 69),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement1Left),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement2Left),
                                                new LowerOuttakeCommand(robot)
                                        ),
                                        new WaitCommand(350),
                                        new InstantCommand(() -> robot.claw.releaseLeft()),
                                        new WaitCommand(350),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement3Left),
                                                new PlusTwoBlueStackCommand(robot)
                                        ),
                                        //                                new InstantCommand(() -> robot.claw.grabBoth()),
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
                    case UNFOUND:
                        TrajectorySequence movement1Right = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(18, 65.50, Math.toRadians(270.00)))
                                .splineToSplineHeading(
                                        new Pose2d(12, 36, Math.toRadians(220.00)), Math.toRadians(220.00),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement2Right = robot.driveSubsystem.trajectorySequenceBuilder(movement1Right.end())
                                .setReversed(true)
                                .splineToSplineHeading(
                                        new Pose2d(54, 31, Math.toRadians(0.00)), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement3Right = robot.driveSubsystem.trajectorySequenceBuilder(movement2Right.end())
                                .setReversed(true)
                                .splineToConstantHeading(
                                        new Vector2d(20, 15), Math.toRadians(180.00),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToConstantHeading(
                                        new Vector2d(-42, 15), Math.toRadians(180.00),
                                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToConstantHeading(
                                        new Vector2d(-58.5, 14.5), Math.toRadians(180.00),
                                        SampleMecanumDrive.getVelocityConstraint(8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .setReversed(false)
                                .build();

                        TrajectorySequence movement4Right = robot.driveSubsystem.trajectorySequenceBuilder(movement3Right.end())
                                .splineToConstantHeading(
                                        new Vector2d(28.98, 14), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement5Right = robot.driveSubsystem.trajectorySequenceBuilder(movement4Right.end())
                                .splineToConstantHeading(
                                        new Vector2d(45, 38), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToConstantHeading(
                                        new Vector2d(50.5, 38), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement6Right = robot.driveSubsystem.trajectorySequenceBuilder(movement5Right.end())
                                .lineToConstantHeading(
                                        new Vector2d(38, 38),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(38, 67),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(57.00, 67),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement1Right),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement2Right),
                                                new LowerOuttakeCommand(robot)
                                        ),
                                        new WaitCommand(350),
                                        new InstantCommand(() -> robot.claw.releaseLeft()),
                                        new WaitCommand(350),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement3Right),
                                                new PlusTwoBlueStackCommand(robot)
                                        ),
                                        //                                new InstantCommand(() -> robot.claw.grabBoth()),
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
                        TrajectorySequence movement1Middle = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(18, 65.50, Math.toRadians(270.00)))
                                .splineToConstantHeading(
                                        new Vector2d(21, 37), Math.toRadians(270.00),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(27.52, 53),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement2Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement1Middle.end())
                                .splineToSplineHeading(
                                        new Pose2d(52.3, 38, Math.toRadians(0.00)), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement3Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement2Middle.end())
                                .setReversed(true)
                                .splineToConstantHeading(
                                        new Vector2d(20, 15), Math.toRadians(180.00),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToConstantHeading(
                                        new Vector2d(-42, 15), Math.toRadians(180.00),
                                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToConstantHeading(
                                        new Vector2d(-58.5, 14.5), Math.toRadians(180.00),
                                        SampleMecanumDrive.getVelocityConstraint(8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .setReversed(false)
                                .build();

                        TrajectorySequence movement4Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement3Middle.end())
                                .splineToConstantHeading(
                                        new Vector2d(28.98, 14), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement5Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement4Middle.end())
                                .splineToConstantHeading(
                                        new Vector2d(45, 38), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToConstantHeading(
                                        new Vector2d(50, 38), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement6Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement5Middle.end())
                                .lineToConstantHeading(
                                        new Vector2d(38, 38),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(38, 15),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(57.00, 15),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement1Middle),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement2Middle),
                                                new LowerOuttakeCommand(robot)
                                        ),
                                        new WaitCommand(350),
                                        new InstantCommand(() -> robot.claw.releaseLeft()),
                                        new WaitCommand(350),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement3Middle),
                                                new PlusTwoBlueStackCommand(robot)
                                        ),
                                        //                                new InstantCommand(() -> robot.claw.grabBoth()),
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
                        TrajectorySequence movement1Left = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(18, 65.50, Math.toRadians(270.00)))
                                .splineToConstantHeading(
                                        new Vector2d(27.8, 43.31), Math.toRadians(270.00),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(27.52, 53),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement2Left = robot.driveSubsystem.trajectorySequenceBuilder(movement1Left.end())
                                .splineToSplineHeading(
                                        new Pose2d(52.3, 41, Math.toRadians(0.00)), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement3Left = robot.driveSubsystem.trajectorySequenceBuilder(movement2Left.end())
                                .setReversed(true)
                                .splineToConstantHeading(
                                        new Vector2d(20, 15), Math.toRadians(180.00),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToConstantHeading(
                                        new Vector2d(-42, 15), Math.toRadians(180.00),
                                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToConstantHeading(
                                        new Vector2d(-58.5, 14.5), Math.toRadians(180.00),
                                        SampleMecanumDrive.getVelocityConstraint(8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .setReversed(false)
                                .build();

                        TrajectorySequence movement4Left = robot.driveSubsystem.trajectorySequenceBuilder(movement3Left.end())
                                .splineToConstantHeading(
                                        new Vector2d(28.98, 14), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement5Left = robot.driveSubsystem.trajectorySequenceBuilder(movement4Left.end())
                                .splineToConstantHeading(
                                        new Vector2d(45, 38), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToConstantHeading(
                                        new Vector2d(50.5, 38), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement6Left = robot.driveSubsystem.trajectorySequenceBuilder(movement5Left.end())
                                .lineToConstantHeading(
                                        new Vector2d(38, 38),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(38, 15),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(57.00, 15),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement1Left),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement2Left),
                                                new LowerOuttakeCommand(robot)
                                        ),
                                        new WaitCommand(350),
                                        new InstantCommand(() -> robot.claw.releaseLeft()),
                                        new WaitCommand(350),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement3Left),
                                                new PlusTwoBlueStackCommand(robot)
                                        ),
                                        //                                new InstantCommand(() -> robot.claw.grabBoth()),
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
                    case UNFOUND:
                        TrajectorySequence movement1Right = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(18, 65.50, Math.toRadians(270.00)))
                                .splineToSplineHeading(
                                        new Pose2d(12, 36, Math.toRadians(220.00)), Math.toRadians(220.00),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement2Right = robot.driveSubsystem.trajectorySequenceBuilder(movement1Right.end())
                                .setReversed(true)
                                .splineToSplineHeading(
                                        new Pose2d(54, 31, Math.toRadians(0.00)), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement3Right = robot.driveSubsystem.trajectorySequenceBuilder(movement2Right.end())
                                .setReversed(true)
                                .splineToConstantHeading(
                                        new Vector2d(20, 15), Math.toRadians(180.00),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToConstantHeading(
                                        new Vector2d(-42, 15), Math.toRadians(180.00),
                                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToConstantHeading(
                                        new Vector2d(-58, 14.5), Math.toRadians(180.00),
                                        SampleMecanumDrive.getVelocityConstraint(8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .setReversed(false)
                                .build();

                        TrajectorySequence movement4Right = robot.driveSubsystem.trajectorySequenceBuilder(movement3Right.end())
                                .splineToConstantHeading(
                                        new Vector2d(28.98, 14), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement5Right = robot.driveSubsystem.trajectorySequenceBuilder(movement4Right.end())
                                .splineToConstantHeading(
                                        new Vector2d(45, 38), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToConstantHeading(
                                        new Vector2d(50.5, 38), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement6Right = robot.driveSubsystem.trajectorySequenceBuilder(movement5Right.end())
                                .lineToConstantHeading(
                                        new Vector2d(38, 38),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(38, 15),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(57.00, 15),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement1Right),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement2Right),
                                                new LowerOuttakeCommand(robot)
                                        ),
                                        new WaitCommand(350),
                                        new InstantCommand(() -> robot.claw.releaseLeft()),
                                        new WaitCommand(350),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement3Right),
                                                new PlusTwoBlueStackCommand(robot)
                                        ),
                                        //                                new InstantCommand(() -> robot.claw.grabBoth()),
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
                        TrajectorySequence movement1Middle = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(18, 65.50, Math.toRadians(270.00)))
                                .splineToConstantHeading(
                                        new Vector2d(21, 37), Math.toRadians(270.00),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(27.52, 53),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement2Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement1Middle.end())
                                .splineToSplineHeading(
                                        new Pose2d(52.3, 36, Math.toRadians(0.00)), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement3Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement2Middle.end())
                                .setReversed(true)
                                .splineToConstantHeading(
                                        new Vector2d(20, 15), Math.toRadians(180.00),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToConstantHeading(
                                        new Vector2d(-42, 15), Math.toRadians(180.00),
                                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToConstantHeading(
                                        new Vector2d(-58, 14.5), Math.toRadians(180.00),
                                        SampleMecanumDrive.getVelocityConstraint(8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .setReversed(false)
                                .build();

                        TrajectorySequence movement4Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement3Middle.end())
                                .splineToConstantHeading(
                                        new Vector2d(28.98, 14), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement5Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement4Middle.end())
                                .splineToConstantHeading(
                                        new Vector2d(45, 38), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToConstantHeading(
                                        new Vector2d(50, 38), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
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
                                        new InstantCommand(() -> robot.claw.releaseLeft()),
                                        new WaitCommand(350),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement3Middle),
                                                new PlusTwoBlueStackCommand(robot)
                                        ),
                                        //                                new InstantCommand(() -> robot.claw.grabBoth()),
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
                        TrajectorySequence movement1Left = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(18, 65.50, Math.toRadians(270.00)))
                                .splineToConstantHeading(
                                        new Vector2d(27.8, 43.31), Math.toRadians(270.00),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(27.52, 53),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement2Left = robot.driveSubsystem.trajectorySequenceBuilder(movement1Left.end())
                                .splineToSplineHeading(
                                        new Pose2d(52.3, 41, Math.toRadians(0.00)), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement3Left = robot.driveSubsystem.trajectorySequenceBuilder(movement2Left.end())
                                .setReversed(true)
                                .splineToConstantHeading(
                                        new Vector2d(20, 15), Math.toRadians(180.00),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToConstantHeading(
                                        new Vector2d(-42, 15), Math.toRadians(180.00),
                                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToConstantHeading(
                                        new Vector2d(-58, 14.5), Math.toRadians(180.00),
                                        SampleMecanumDrive.getVelocityConstraint(8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .setReversed(false)
                                .build();

                        TrajectorySequence movement4Left = robot.driveSubsystem.trajectorySequenceBuilder(movement3Left.end())
                                .splineToConstantHeading(
                                        new Vector2d(28.98, 14), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement5Left = robot.driveSubsystem.trajectorySequenceBuilder(movement4Left.end())
                                .splineToConstantHeading(
                                        new Vector2d(45, 38), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToConstantHeading(
                                        new Vector2d(50.5, 38), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
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
                                                new LowerOuttakeCommand(robot)
                                        ),
                                        new WaitCommand(350),
                                        new InstantCommand(() -> robot.claw.releaseLeft()),
                                        new WaitCommand(350),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement3Left),
                                                new PlusTwoBlueStackCommand(robot)
                                        ),
                                        //                                new InstantCommand(() -> robot.claw.grabBoth()),
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
                    case UNFOUND:
                        TrajectorySequence movement1Right = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(18, 65.50, Math.toRadians(270.00)))
                                .splineToSplineHeading(
                                        new Pose2d(12, 36, Math.toRadians(220.00)), Math.toRadians(220.00),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement2Right = robot.driveSubsystem.trajectorySequenceBuilder(movement1Right.end())
                                .setReversed(true)
                                .splineToSplineHeading(
                                        new Pose2d(54, 31, Math.toRadians(0.00)), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement3Right = robot.driveSubsystem.trajectorySequenceBuilder(movement2Right.end())
                                .setReversed(true)
                                .splineToConstantHeading(
                                        new Vector2d(20, 15), Math.toRadians(180.00),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToConstantHeading(
                                        new Vector2d(-42, 15), Math.toRadians(180.00),
                                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToConstantHeading(
                                        new Vector2d(-58, 14.5), Math.toRadians(180.00),
                                        SampleMecanumDrive.getVelocityConstraint(8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .setReversed(false)
                                .build();

                        TrajectorySequence movement4Right = robot.driveSubsystem.trajectorySequenceBuilder(movement3Right.end())
                                .splineToConstantHeading(
                                        new Vector2d(28.98, 14), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement5Right = robot.driveSubsystem.trajectorySequenceBuilder(movement4Right.end())
                                .splineToConstantHeading(
                                        new Vector2d(45, 38), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToConstantHeading(
                                        new Vector2d(50.5, 38), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
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
                                                new LowerOuttakeCommand(robot)
                                        ),
                                        new WaitCommand(350),
                                        new InstantCommand(() -> robot.claw.releaseLeft()),
                                        new WaitCommand(350),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement3Right),
                                                new PlusTwoBlueStackCommand(robot)
                                        ),
                                        //                                new InstantCommand(() -> robot.claw.grabBoth()),
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
        } else if (!plusTwo) { //REGULAR 50
            if (parkLeft) {
                switch (recordedPropPosition) {
                    case LEFT:
                        TrajectorySequence movement1Left = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(16.14, 63.32, Math.toRadians(-90.00)))
                                .splineToConstantHeading(
                                        new Vector2d(27.8, 43.31), Math.toRadians(270.00),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(27.52, 53),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement2Left = robot.driveSubsystem.trajectorySequenceBuilder(movement1Left.end())
                                .splineToSplineHeading(
                                        new Pose2d(52.3, 41, Math.toRadians(0.00)), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement3Left = robot.driveSubsystem.trajectorySequenceBuilder(movement2Left.end())
                                .lineToConstantHeading(
                                        new Vector2d(38.41, 42),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(39.06, 62),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(58, 62),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement1Left),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement2Left),
                                                new LowerOuttakeCommand(robot)
                                        ),
                                        new InstantCommand(() -> robot.claw.releaseLeft()),
                                        new WaitCommand(750),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement3Left),
                                                new RestCommand(robot)
                                        )

                                )
                        );
                        break;
                    case RIGHT:
                    case UNFOUND:
                        TrajectorySequence movement1Right = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(16.14, 63.32, Math.toRadians(-90.00)))
                                .splineToSplineHeading(
                                        new Pose2d(10, 36, Math.toRadians(220.00)), Math.toRadians(220.00),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement2Right = robot.driveSubsystem.trajectorySequenceBuilder(movement1Right.end())
                                .setReversed(true)
                                .splineToSplineHeading(
                                        new Pose2d(52.7, 30, Math.toRadians(0.00)), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement3Right = robot.driveSubsystem.trajectorySequenceBuilder(movement2Right.end())
                                .lineToConstantHeading(
                                        new Vector2d(38.41, 28.4),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(39.06, 62),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(58, 62),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement1Right),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement2Right),
                                                new LowerOuttakeCommand(robot)
                                        ),
                                        new InstantCommand(() -> robot.claw.releaseLeft()),
                                        new WaitCommand(750),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement3Right),
                                                new RestCommand(robot)
                                        )

                                )
                        );
                        break;
                    case MIDDLE:
                        TrajectorySequence movement1Middle = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(16.14, 63.32, Math.toRadians(-90.00)))
                                .splineToConstantHeading(
                                        new Vector2d(20.25, 33.88), Math.toRadians(270.00),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(20.25, 45),
                                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement2Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement1Middle.end())
                                .splineToSplineHeading(
                                        new Pose2d(52.6, 35, Math.toRadians(0.00)), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement3Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement2Middle.end())
                                .lineToConstantHeading(
                                        new Vector2d(38.41, 35),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(39.06, 62),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(58, 62),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement1Middle),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement2Middle),
                                                new LowerOuttakeCommand(robot)
                                        ),
                                        new InstantCommand(() -> robot.claw.releaseLeft()),
                                        new WaitCommand(200),
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
                        TrajectorySequence movement1Left = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(16.14, 63.32, Math.toRadians(-90.00)))
                                .splineToConstantHeading(
                                        new Vector2d(27.8, 43.31), Math.toRadians(270.00),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(27.52, 53),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement2Left = robot.driveSubsystem.trajectorySequenceBuilder(movement1Left.end())
                                .splineToSplineHeading(
                                        new Pose2d(52.3, 41, Math.toRadians(0.00)), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement3Left = robot.driveSubsystem.trajectorySequenceBuilder(movement2Left.end())
                                .lineToConstantHeading(
                                        new Vector2d(38.41, 42),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(39.06, 15),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(58, 15),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement1Left),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement2Left),
                                                new LowerOuttakeCommand(robot)
                                        ),
                                        new InstantCommand(() -> robot.claw.releaseLeft()),
                                        new WaitCommand(750),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement3Left),
                                                new RestCommand(robot)
                                        )

                                )
                        );
                        break;
                    case RIGHT:
                    case UNFOUND:
                        TrajectorySequence movement1Right = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(16.14, 63.32, Math.toRadians(-90.00)))
                                .splineToSplineHeading(
                                        new Pose2d(10, 36, Math.toRadians(220.00)), Math.toRadians(220.00),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement2Right = robot.driveSubsystem.trajectorySequenceBuilder(movement1Right.end())
                                .setReversed(true)
                                .splineToSplineHeading(
                                        new Pose2d(52.7, 30, Math.toRadians(0.00)), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement3Right = robot.driveSubsystem.trajectorySequenceBuilder(movement2Right.end())
                                .lineToConstantHeading(
                                        new Vector2d(38.41, 28.4),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(39.06, 13),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(58, 13),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement1Right),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement2Right),
                                                new LowerOuttakeCommand(robot)
                                        ),
                                        new InstantCommand(() -> robot.claw.releaseLeft()),
                                        new WaitCommand(750),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement3Right),
                                                new RestCommand(robot)
                                        )

                                )
                        );
                        break;
                    case MIDDLE:
                        TrajectorySequence movement1Middle = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(16.14, 63.32, Math.toRadians(-90.00)))
                                .splineToConstantHeading(
                                        new Vector2d(20.25, 33.88), Math.toRadians(270.00),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(20.25, 45),
                                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement2Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement1Middle.end())
                                .splineToSplineHeading(
                                        new Pose2d(52.6, 35, Math.toRadians(0.00)), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement3Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement2Middle.end())
                                .lineToConstantHeading(
                                        new Vector2d(38.41, 35),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(39.06, 13),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(58, 13),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement1Middle),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement2Middle),
                                                new LowerOuttakeCommand(robot)
                                        ),
                                        new InstantCommand(() -> robot.claw.releaseLeft()),
                                        new WaitCommand(200),
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
                        TrajectorySequence movement1Left = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(16.14, 63.32, Math.toRadians(-90.00)))
                                .splineToConstantHeading(
                                        new Vector2d(27.8, 43.31), Math.toRadians(270.00),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(27.52, 53),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement2Left = robot.driveSubsystem.trajectorySequenceBuilder(movement1Left.end())
                                .splineToSplineHeading(
                                        new Pose2d(52.3, 41, Math.toRadians(0.00)), Math.toRadians(0.00),
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
                                                new LowerOuttakeCommand(robot)
                                        ),
                                        new InstantCommand(() -> robot.claw.releaseLeft()),
                                        new WaitCommand(750),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement3Left),
                                                new RestCommand(robot)
                                        )

                                )
                        );
                        break;
                    case RIGHT:
                    case UNFOUND:
                        TrajectorySequence movement1Right = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(16.14, 63.32, Math.toRadians(-90.00)))
                                .splineToSplineHeading(
                                        new Pose2d(10, 36, Math.toRadians(220.00)), Math.toRadians(220.00),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement2Right = robot.driveSubsystem.trajectorySequenceBuilder(movement1Right.end())
                                .setReversed(true)
                                .splineToSplineHeading(
                                        new Pose2d(52.7, 30, Math.toRadians(0.00)), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
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
                                                new LowerOuttakeCommand(robot)
                                        ),
                                        new InstantCommand(() -> robot.claw.releaseLeft()),
                                        new WaitCommand(750),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement3Right),
                                                new RestCommand(robot)
                                        )

                                )
                        );
                        break;
                    case MIDDLE:
                        TrajectorySequence movement1Middle = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(16.14, 63.32, Math.toRadians(-90.00)))
                                .splineToConstantHeading(
                                        new Vector2d(20.25, 33.88), Math.toRadians(270.00),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(20.25, 45),
                                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement2Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement1Middle.end())
                                .splineToSplineHeading(
                                        new Pose2d(52.6, 35, Math.toRadians(0.00)), Math.toRadians(0.00),
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
                                                new LowerOuttakeCommand(robot)
                                        ),
                                        new InstantCommand(() -> robot.claw.releaseLeft()),
                                        new WaitCommand(200),
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