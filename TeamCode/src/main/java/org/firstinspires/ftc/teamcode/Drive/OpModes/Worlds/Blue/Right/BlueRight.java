package org.firstinspires.ftc.teamcode.Drive.OpModes.Worlds.Blue.Right;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.TrajectorySequences.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.DriveCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.LowOuttakeCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.PlusOneBlueStackCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.RestCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.SecondOuttakeCommand;
import org.firstinspires.ftc.teamcode.Utility.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.RoadRunner.DriveConstants;
import org.firstinspires.ftc.teamcode.Utility.RoadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Utility.Vision.Prop.NewBlueRightProcessor;
import org.firstinspires.ftc.teamcode.Utility.Vision.Robot.Center.BlueCenterRobotScan;
import org.firstinspires.ftc.teamcode.Utility.Vision.Robot.Wall.BlueWallRobotScan;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class BlueRight extends OpMode {
    private RobotHardware robot;
    boolean previousX, currentX;
    boolean parkLeft = false, parkBoard = false, parkRight = true;
    boolean previousY, currentY;
    boolean plusOne = false;
    boolean previousB, currentB;
    boolean truss = false, center = true;

    private VisionPortal visionPortal;
    private NewBlueRightProcessor colorMassDetectionProcessor;
    private BlueWallRobotScan robotProcessor;
    private BlueCenterRobotScan robotProcessor2;

    private ElapsedTime time_since_start;
    private double loop;
    boolean robotSensedWall;
    boolean robotSensed;

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

        colorMassDetectionProcessor = new NewBlueRightProcessor();
        robotProcessor = new BlueWallRobotScan();
        robotProcessor2 = new BlueCenterRobotScan();

        colorMassDetectionProcessor.setDetectionColor(false); //false is blue, true is red
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .addProcessors(colorMassDetectionProcessor, robotProcessor, robotProcessor2)
                .build();
    }

    @Override
    public void init_loop() {
        currentX = gamepad1.x;
        currentY = gamepad1.y;
        currentB = gamepad1.b;

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
            if (plusOne) {
                plusOne = false;
            } else if (!plusOne) {
                plusOne = true;
            }
        }

        telemetry.addData("Autonomous (Triangle): ", plusOne ? "Plus One" : "Regular 50");
        if (!plusOne) {
            if (currentB && !previousB) {
                if (truss) {
                    truss = false;
                    center = true;
                } else if (center) {
                    truss = true;
                    center = false;
                }
            }

            telemetry.addData("Getting to Backboard Using (Circle):", truss ? "Truss/Wall" : center ? "Center" : "");
        }
        telemetry.addData("Parking On (Square): ", parkLeft ? "Left" : parkRight ? "Right" : parkBoard ? "At Board" : "");
        telemetry.addData("Currently Recorded Position: ", colorMassDetectionProcessor.getPropLocation());
        telemetry.addData("Camera State: ", visionPortal.getCameraState());
        telemetry.update();
        CommandScheduler.getInstance().run();
        robot.armSystem.loop();

        previousX = currentX;
        previousY = currentY;
        previousB = currentB;
    }

    @Override
    public void start() {
        time_since_start = new ElapsedTime();

        FtcDashboard.getInstance().startCameraStream(robotProcessor, 30);

        NewBlueRightProcessor.PropPositions recordedPropPosition = colorMassDetectionProcessor.getPropLocation();
        robot.driveSubsystem.setPoseEstimate(new Pose2d(-40.11, 63.48, Math.toRadians(-90.00)));

        if (plusOne) { //PLUSONE
            if (parkBoard) { //PLUS ONE BOARD
                switch (recordedPropPosition) {
                    case RIGHT:
                        TrajectorySequence movement1Right = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, 63.48, Math.toRadians(-90.00)))
                                .splineToConstantHeading(
                                        new Vector2d(-50.6, 40), Math.toRadians(-90.00),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToSplineHeading(
                                        new Pose2d(-57.5, 55.39, Math.toRadians(0)),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(-57.5, 38.5),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build(); //drop arm to intake open claw

                        TrajectorySequence lilMoreCuh = robot.driveSubsystem.trajectorySequenceBuilder(movement1Right.end())
                                .lineToConstantHeading(
                                        new Vector2d(-60, 38.5),
                                        SampleMecanumDrive.getVelocityConstraint(3, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement2Right = robot.driveSubsystem.trajectorySequenceBuilder(lilMoreCuh.end())
                                .lineToConstantHeading(new Vector2d(-58, 39))
                                .splineToConstantHeading(
                                        new Vector2d(-47.77, 60.5), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToConstantHeading(
                                        new Vector2d(6.67, 61), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .turn(Math.toRadians(-30))
                                .build();

                        TrajectorySequence movement3Right = robot.driveSubsystem.trajectorySequenceBuilder(movement2Right.end())
                                .lineToSplineHeading(new Pose2d(20, 60.5, Math.toRadians(0.00)))
                                .splineToConstantHeading(
                                        new Vector2d(50, 32), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement4Right = robot.driveSubsystem.trajectorySequenceBuilder(movement3Right.end())
                                .setTangent(-180)
                                .splineToConstantHeading(
                                        new Vector2d(
                                                47, 44), Math.toRadians(0),
                                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement5Right = robot.driveSubsystem.trajectorySequenceBuilder(movement4Right.end())
                                .back(6)
                                .build();

                        CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
//                                        new ParallelCommandGroup(
//                                                new PlusOneBlueStackCommand(robot),
//                                                new DriveCommand(robot.driveSubsystem, movement1Right)
//                                        ),
                                        new DriveCommand(robot.driveSubsystem, movement1Right),
                                        new PlusOneBlueStackCommand(robot),
                                        new WaitCommand(500),
                                        new DriveCommand(robot.driveSubsystem, lilMoreCuh),
                                        new WaitCommand(500),
                                        new InstantCommand(() -> robot.claw.grabBoth()),
                                        new WaitCommand(500),
                                        new DriveCommand(robot.driveSubsystem, movement2Right),
                                        new WaitUntilCommand(() -> robotSensedWall || time_since_start.seconds() > 18),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement3Right),
                                                new LowOuttakeCommand(robot)
                                        ),
                                        new WaitCommand(350),
                                        new InstantCommand(() -> robot.claw.releaseLeft()),
                                        new WaitCommand(350),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement4Right),
                                                new SecondOuttakeCommand(robot)
                                        ),
                                        new WaitCommand(600),
                                        new InstantCommand(() -> robot.claw.releaseRight()),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement5Right),
                                                new RestCommand(robot)
                                        )

                                )
                        );
                        break;
                    case LEFT:
                    case UNFOUND:
                        TrajectorySequence movement1Left = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, 63.48, Math.toRadians(-90.00)))
                                .splineTo(
                                        new Vector2d(-36, 39), Math.toRadians(-45.00),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToSplineHeading(
                                        new Pose2d(-57, 55.39, Math.toRadians(0)),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(-57, 38.5),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build(); //drop arm to intake open claw

                        TrajectorySequence lilMoreCuh2 = robot.driveSubsystem.trajectorySequenceBuilder(movement1Left.end())
                                .lineToConstantHeading(
                                        new Vector2d(-60, 38.5),
                                        SampleMecanumDrive.getVelocityConstraint(3, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement2Left = robot.driveSubsystem.trajectorySequenceBuilder(lilMoreCuh2.end())
                                .lineToConstantHeading(new Vector2d(-57.60, 39))
                                .splineToConstantHeading(
                                        new Vector2d(-47.77, 60.5), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToConstantHeading(
                                        new Vector2d(6.67, 60.5), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .turn(Math.toRadians(-30))
                                .build();

                        TrajectorySequence movement3Left = robot.driveSubsystem.trajectorySequenceBuilder(movement2Left.end())
                                .lineToSplineHeading(new Pose2d(20, 60.5, Math.toRadians(0.00)))
                                .splineToConstantHeading(
                                        new Vector2d(50, 43.5), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement4Left = robot.driveSubsystem.trajectorySequenceBuilder(movement3Left.end())
                                .setTangent(-180)
                                .splineToConstantHeading(
                                        new Vector2d(
                                                47, 35.55), Math.toRadians(0),
                                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement5Left = robot.driveSubsystem.trajectorySequenceBuilder(movement4Left.end())
                                .back(6)
                                .build();

                        CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement1Left),
                                        new PlusOneBlueStackCommand(robot),
                                        new WaitCommand(500),
                                        new DriveCommand(robot.driveSubsystem, lilMoreCuh2),
                                        new WaitCommand(500),
                                        new InstantCommand(() -> robot.claw.grabBoth()),
                                        new WaitCommand(500),
                                        new DriveCommand(robot.driveSubsystem, movement2Left),
                                        new WaitUntilCommand(() -> robotSensedWall || time_since_start.seconds() > 18),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement3Left),
                                                new LowOuttakeCommand(robot)
                                        ),
                                        new WaitCommand(350),
                                        new InstantCommand(() -> robot.claw.releaseLeft()),
                                        new WaitCommand(350),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement4Left),
                                                new SecondOuttakeCommand(robot)
                                        ),
                                        new WaitCommand(600),
                                        new InstantCommand(() -> robot.claw.releaseRight()),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement5Left),
                                                new RestCommand(robot)
                                        )

                                )
                        );
                        break;
                    case MIDDLE:
                        TrajectorySequence movement1Middle = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, 63.48, Math.toRadians(-90.00)))
                                .splineToConstantHeading(
                                        new Vector2d(-44.00, 34.3), Math.toRadians(-90.00),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToSplineHeading(
                                        new Pose2d(-57.5, 55.39, Math.toRadians(0)),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(-57.5, 38.5),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build(); //drop arm to intake open claw

                        TrajectorySequence lilMoreCuh3 = robot.driveSubsystem.trajectorySequenceBuilder(movement1Middle.end())
                                .lineToConstantHeading(
                                        new Vector2d(-60, 38.5),
                                        SampleMecanumDrive.getVelocityConstraint(3, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement2Middle = robot.driveSubsystem.trajectorySequenceBuilder(lilMoreCuh3.end())
                                .lineToConstantHeading(new Vector2d(-57.60, 39))
                                .splineToConstantHeading(
                                        new Vector2d(-47.77, 60.5), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToConstantHeading(
                                        new Vector2d(6.67, 61), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .turn(Math.toRadians(-30))
                                .build();

                        TrajectorySequence movement3Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement2Middle.end())
                                .lineToSplineHeading(new Pose2d(20, 60.5, Math.toRadians(0.00)))
                                .splineToConstantHeading(
                                        new Vector2d(50, 39.5), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement4Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement3Middle.end())
                                .setTangent(-180)
                                .splineToConstantHeading(
                                        new Vector2d(
                                                47, 35.55), Math.toRadians(0),
                                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement5Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement4Middle.end())
                                .back(6)
                                .build();

                        CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement1Middle),
                                        new PlusOneBlueStackCommand(robot),
                                        new WaitCommand(500),
                                        new DriveCommand(robot.driveSubsystem, lilMoreCuh3),
                                        new WaitCommand(500),
                                        new InstantCommand(() -> robot.claw.grabBoth()),
                                        new WaitCommand(500),
                                        new DriveCommand(robot.driveSubsystem, movement2Middle),
                                        new WaitUntilCommand(() -> robotSensedWall || time_since_start.seconds() > 18),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement3Middle),
                                                new LowOuttakeCommand(robot)
                                        ),
                                        new WaitCommand(350),
                                        new InstantCommand(() -> robot.claw.releaseLeft()),
                                        new WaitCommand(350),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement4Middle),
                                                new SecondOuttakeCommand(robot)
                                        ),
                                        new WaitCommand(600),
                                        new InstantCommand(() -> robot.claw.releaseRight()),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement5Middle),
                                                new RestCommand(robot)
                                        )

                                )
                        );

                        break;
                }
            } else if (parkRight) { //PLUS ONE RIGHT
                switch (recordedPropPosition) {
                    case RIGHT:
                        TrajectorySequence movement1Right = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, 63.48, Math.toRadians(-90.00)))
                                .splineToConstantHeading(
                                        new Vector2d(-50.6, 40), Math.toRadians(-90.00),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToSplineHeading(
                                        new Pose2d(-57.5, 55.39, Math.toRadians(0)),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(-57.5, 38.5),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build(); //drop arm to intake open claw

                        TrajectorySequence lilMoreCuh = robot.driveSubsystem.trajectorySequenceBuilder(movement1Right.end())
                                .lineToConstantHeading(
                                        new Vector2d(-60, 38.5),
                                        SampleMecanumDrive.getVelocityConstraint(3, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement2Right = robot.driveSubsystem.trajectorySequenceBuilder(lilMoreCuh.end())
                                .lineToConstantHeading(new Vector2d(-58, 39))
                                .splineToConstantHeading(
                                        new Vector2d(-47.77, 60.5), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToConstantHeading(
                                        new Vector2d(6.67, 61), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .turn(Math.toRadians(-30))
                                .build();

                        TrajectorySequence movement3Right = robot.driveSubsystem.trajectorySequenceBuilder(movement2Right.end())
                                .lineToSplineHeading(new Pose2d(20, 60.5, Math.toRadians(0.00)))
                                .splineToConstantHeading(
                                        new Vector2d(50, 32), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement4Right = robot.driveSubsystem.trajectorySequenceBuilder(movement3Right.end())
                                .setTangent(-180)
                                .splineToConstantHeading(
                                        new Vector2d(
                                                47, 44), Math.toRadians(0),
                                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement5Right = robot.driveSubsystem.trajectorySequenceBuilder(movement4Right.end())
                                .lineToConstantHeading(
                                        new Vector2d(37, 44),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(37, 10),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(54, 10),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement1Right),
                                        new PlusOneBlueStackCommand(robot),
                                        new WaitCommand(500),
                                        new DriveCommand(robot.driveSubsystem, lilMoreCuh),
                                        new WaitCommand(500),
                                        new InstantCommand(() -> robot.claw.grabBoth()),
                                        new WaitCommand(500),
                                        new DriveCommand(robot.driveSubsystem, movement2Right),
                                        new WaitUntilCommand(() -> robotSensedWall || time_since_start.seconds() > 18),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement3Right),
                                                new LowOuttakeCommand(robot)
                                        ),
                                        new WaitCommand(350),
                                        new InstantCommand(() -> robot.claw.releaseLeft()),
                                        new WaitCommand(350),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement4Right),
                                                new SecondOuttakeCommand(robot)
                                        ),
                                        new WaitCommand(600),
                                        new InstantCommand(() -> robot.claw.releaseRight()),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement5Right),
                                                new RestCommand(robot)
                                        )

                                )
                        );
                        break;
                    case LEFT:
                    case UNFOUND:
                        TrajectorySequence movement1Left = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, 63.48, Math.toRadians(-90.00)))
                                .splineTo(
                                        new Vector2d(-36, 39), Math.toRadians(-45.00),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToSplineHeading(
                                        new Pose2d(-57, 55.39, Math.toRadians(0)),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(-57, 38.5),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build(); //drop arm to intake open claw

                        TrajectorySequence lilMoreCuh2 = robot.driveSubsystem.trajectorySequenceBuilder(movement1Left.end())
                                .lineToConstantHeading(
                                        new Vector2d(-60, 38.5),
                                        SampleMecanumDrive.getVelocityConstraint(3, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement2Left = robot.driveSubsystem.trajectorySequenceBuilder(lilMoreCuh2.end())
                                .lineToConstantHeading(new Vector2d(-57.60, 39))
                                .splineToConstantHeading(
                                        new Vector2d(-47.77, 60.5), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToConstantHeading(
                                        new Vector2d(6.67, 60.5), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .turn(Math.toRadians(-30))
                                .build();

                        TrajectorySequence movement3Left = robot.driveSubsystem.trajectorySequenceBuilder(movement2Left.end())
                                .lineToSplineHeading(new Pose2d(20, 60.5, Math.toRadians(0.00)))
                                .splineToConstantHeading(
                                        new Vector2d(50, 43.5), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement4Left = robot.driveSubsystem.trajectorySequenceBuilder(movement3Left.end())
                                .setTangent(-180)
                                .splineToConstantHeading(
                                        new Vector2d(
                                                47, 35.55), Math.toRadians(0),
                                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement5Left = robot.driveSubsystem.trajectorySequenceBuilder(movement4Left.end())
                                .lineToConstantHeading(
                                        new Vector2d(37, 35.55),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(37, 10),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(54, 10),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement1Left),
                                        new PlusOneBlueStackCommand(robot),
                                        new WaitCommand(500),
                                        new DriveCommand(robot.driveSubsystem, lilMoreCuh2),
                                        new WaitCommand(500),
                                        new InstantCommand(() -> robot.claw.grabBoth()),
                                        new WaitCommand(500),
                                        new DriveCommand(robot.driveSubsystem, movement2Left),
                                        new WaitUntilCommand(() -> robotSensedWall || time_since_start.seconds() > 18),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement3Left),
                                                new LowOuttakeCommand(robot)
                                        ),
                                        new WaitCommand(350),
                                        new InstantCommand(() -> robot.claw.releaseLeft()),
                                        new WaitCommand(350),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement4Left),
                                                new SecondOuttakeCommand(robot)
                                        ),
                                        new WaitCommand(600),
                                        new InstantCommand(() -> robot.claw.releaseRight()),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement5Left),
                                                new RestCommand(robot)
                                        )

                                )
                        );
                        break;
                    case MIDDLE:
                        TrajectorySequence movement1Middle = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, 63.48, Math.toRadians(-90.00)))
                                .splineToConstantHeading(
                                        new Vector2d(-44.00, 34.3), Math.toRadians(-90.00),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToSplineHeading(
                                        new Pose2d(-57.5, 55.39, Math.toRadians(0)),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(-57.5, 38.5),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build(); //drop arm to intake open claw

                        TrajectorySequence lilMoreCuh3 = robot.driveSubsystem.trajectorySequenceBuilder(movement1Middle.end())
                                .lineToConstantHeading(
                                        new Vector2d(-60, 38.5),
                                        SampleMecanumDrive.getVelocityConstraint(3, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement2Middle = robot.driveSubsystem.trajectorySequenceBuilder(lilMoreCuh3.end())
                                .lineToConstantHeading(new Vector2d(-57.60, 39))
                                .splineToConstantHeading(
                                        new Vector2d(-47.77, 60.5), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToConstantHeading(
                                        new Vector2d(6.67, 61), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .turn(Math.toRadians(-30))
                                .build();

                        TrajectorySequence movement3Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement2Middle.end())
                                .lineToSplineHeading(new Pose2d(20, 60.5, Math.toRadians(0.00)))
                                .splineToConstantHeading(
                                        new Vector2d(50, 39.5), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement4Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement3Middle.end())
                                .setTangent(-180)
                                .splineToConstantHeading(
                                        new Vector2d(
                                                47, 35.55), Math.toRadians(0),
                                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement5Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement4Middle.end())
                                .lineToConstantHeading(
                                        new Vector2d(37, 35.55),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(37, 10),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(54, 10),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement1Middle),
                                        new PlusOneBlueStackCommand(robot),
                                        new WaitCommand(500),
                                        new DriveCommand(robot.driveSubsystem, lilMoreCuh3),
                                        new WaitCommand(500),
                                        new InstantCommand(() -> robot.claw.grabBoth()),
                                        new WaitCommand(500),
                                        new DriveCommand(robot.driveSubsystem, movement2Middle),
                                        new WaitUntilCommand(() -> robotSensedWall || time_since_start.seconds() > 18),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement3Middle),
                                                new LowOuttakeCommand(robot)
                                        ),
                                        new WaitCommand(350),
                                        new InstantCommand(() -> robot.claw.releaseLeft()),
                                        new WaitCommand(350),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement4Middle),
                                                new SecondOuttakeCommand(robot)
                                        ),
                                        new WaitCommand(600),
                                        new InstantCommand(() -> robot.claw.releaseRight()),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement5Middle),
                                                new RestCommand(robot)
                                        )

                                )
                        );

                        break;
                }
            } else if (parkLeft) { //PLUS ONE LEFT
                switch (recordedPropPosition) {
                    case RIGHT:
                        TrajectorySequence movement1Right = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, 63.48, Math.toRadians(-90.00)))
                                .splineToConstantHeading(
                                        new Vector2d(-50.6, 40), Math.toRadians(-90.00),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToSplineHeading(
                                        new Pose2d(-57.5, 55.39, Math.toRadians(0)),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(-57.5, 38.5),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build(); //drop arm to intake open claw

                        TrajectorySequence lilMoreCuh = robot.driveSubsystem.trajectorySequenceBuilder(movement1Right.end())
                                .lineToConstantHeading(
                                        new Vector2d(-60, 38.5),
                                        SampleMecanumDrive.getVelocityConstraint(3, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement2Right = robot.driveSubsystem.trajectorySequenceBuilder(lilMoreCuh.end())
                                .lineToConstantHeading(new Vector2d(-58, 39))
                                .splineToConstantHeading(
                                        new Vector2d(-47.77, 60.5), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToConstantHeading(
                                        new Vector2d(6.67, 61), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .turn(Math.toRadians(-30))
                                .build();

                        TrajectorySequence movement3Right = robot.driveSubsystem.trajectorySequenceBuilder(movement2Right.end())
                                .lineToSplineHeading(new Pose2d(20, 60.5, Math.toRadians(0.00)))
                                .splineToConstantHeading(
                                        new Vector2d(50, 32), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement4Right = robot.driveSubsystem.trajectorySequenceBuilder(movement3Right.end())
                                .setTangent(-180)
                                .splineToConstantHeading(
                                        new Vector2d(
                                                47, 44), Math.toRadians(0),
                                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement5Right = robot.driveSubsystem.trajectorySequenceBuilder(movement4Right.end())
                                .lineToConstantHeading(
                                        new Vector2d(37, 44),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(37, 63),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(54, 63),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement1Right),
                                        new PlusOneBlueStackCommand(robot),
                                        new WaitCommand(500),
                                        new DriveCommand(robot.driveSubsystem, lilMoreCuh),
                                        new WaitCommand(500),
                                        new InstantCommand(() -> robot.claw.grabBoth()),
                                        new WaitCommand(500),
                                        new DriveCommand(robot.driveSubsystem, movement2Right),
                                        new WaitUntilCommand(() -> robotSensedWall || time_since_start.seconds() > 18),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement3Right),
                                                new LowOuttakeCommand(robot)
                                        ),
                                        new WaitCommand(350),
                                        new InstantCommand(() -> robot.claw.releaseLeft()),
                                        new WaitCommand(350),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement4Right),
                                                new SecondOuttakeCommand(robot)
                                        ),
                                        new WaitCommand(600),
                                        new InstantCommand(() -> robot.claw.releaseRight()),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement5Right),
                                                new RestCommand(robot)
                                        )

                                )
                        );
                        break;
                    case LEFT:
                    case UNFOUND:
                        TrajectorySequence movement1Left = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, 63.48, Math.toRadians(-90.00)))
                                .splineTo(
                                        new Vector2d(-36, 39), Math.toRadians(-45.00),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToSplineHeading(
                                        new Pose2d(-57, 55.39, Math.toRadians(0)),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(-57, 38.5),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build(); //drop arm to intake open claw

                        TrajectorySequence lilMoreCuh2 = robot.driveSubsystem.trajectorySequenceBuilder(movement1Left.end())
                                .lineToConstantHeading(
                                        new Vector2d(-60, 38.5),
                                        SampleMecanumDrive.getVelocityConstraint(3, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement2Left = robot.driveSubsystem.trajectorySequenceBuilder(lilMoreCuh2.end())
                                .lineToConstantHeading(new Vector2d(-57.60, 39))
                                .splineToConstantHeading(
                                        new Vector2d(-47.77, 60.5), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToConstantHeading(
                                        new Vector2d(6.67, 60.5), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .turn(Math.toRadians(-30))
                                .build();

                        TrajectorySequence movement3Left = robot.driveSubsystem.trajectorySequenceBuilder(movement2Left.end())
                                .lineToSplineHeading(new Pose2d(20, 60.5, Math.toRadians(0.00)))
                                .splineToConstantHeading(
                                        new Vector2d(50, 43.5), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement4Left = robot.driveSubsystem.trajectorySequenceBuilder(movement3Left.end())
                                .setTangent(-180)
                                .splineToConstantHeading(
                                        new Vector2d(
                                                47, 35.55), Math.toRadians(0),
                                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement5Left = robot.driveSubsystem.trajectorySequenceBuilder(movement4Left.end())
                                .lineToConstantHeading(
                                        new Vector2d(37, 35.55),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(37, 63),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(54, 63),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement1Left),
                                        new PlusOneBlueStackCommand(robot),
                                        new WaitCommand(500),
                                        new DriveCommand(robot.driveSubsystem, lilMoreCuh2),
                                        new WaitCommand(500),
                                        new InstantCommand(() -> robot.claw.grabBoth()),
                                        new WaitCommand(500),
                                        new DriveCommand(robot.driveSubsystem, movement2Left),
                                        new WaitUntilCommand(() -> robotSensedWall || time_since_start.seconds() > 18),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement3Left),
                                                new LowOuttakeCommand(robot)
                                        ),
                                        new WaitCommand(350),
                                        new InstantCommand(() -> robot.claw.releaseLeft()),
                                        new WaitCommand(350),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement4Left),
                                                new SecondOuttakeCommand(robot)
                                        ),
                                        new WaitCommand(600),
                                        new InstantCommand(() -> robot.claw.releaseRight()),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement5Left),
                                                new RestCommand(robot)
                                        )

                                )
                        );
                        break;
                    case MIDDLE:
                        TrajectorySequence movement1Middle = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, 63.48, Math.toRadians(-90.00)))
                                .splineToConstantHeading(
                                        new Vector2d(-44.00, 34.3), Math.toRadians(-90.00),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToSplineHeading(
                                        new Pose2d(-57.5, 55.39, Math.toRadians(0)),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(-57.5, 38.5),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build(); //drop arm to intake open claw

                        TrajectorySequence lilMoreCuh3 = robot.driveSubsystem.trajectorySequenceBuilder(movement1Middle.end())
                                .lineToConstantHeading(
                                        new Vector2d(-60, 38.5),
                                        SampleMecanumDrive.getVelocityConstraint(3, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement2Middle = robot.driveSubsystem.trajectorySequenceBuilder(lilMoreCuh3.end())
                                .lineToConstantHeading(new Vector2d(-57.60, 39))
                                .splineToConstantHeading(
                                        new Vector2d(-47.77, 60.5), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToConstantHeading(
                                        new Vector2d(6.67, 61), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .turn(Math.toRadians(-30))
                                .build();

                        TrajectorySequence movement3Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement2Middle.end())
                                .lineToSplineHeading(new Pose2d(20, 60.5, Math.toRadians(0.00)))
                                .splineToConstantHeading(
                                        new Vector2d(50, 39.5), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement4Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement3Middle.end())
                                .setTangent(-180)
                                .splineToConstantHeading(
                                        new Vector2d(
                                                47, 35.55), Math.toRadians(0),
                                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement5Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement4Middle.end())
                                .lineToConstantHeading(
                                        new Vector2d(37, 35.55),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(37, 63),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(54, 63),
                                        SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement1Middle),
                                        new PlusOneBlueStackCommand(robot),
                                        new WaitCommand(500),
                                        new DriveCommand(robot.driveSubsystem, lilMoreCuh3),
                                        new WaitCommand(500),
                                        new InstantCommand(() -> robot.claw.grabBoth()),
                                        new WaitCommand(500),
                                        new DriveCommand(robot.driveSubsystem, movement2Middle),
                                        new WaitUntilCommand(() -> robotSensedWall || time_since_start.seconds() > 18),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement3Middle),
                                                new LowOuttakeCommand(robot)
                                        ),
                                        new WaitCommand(350),
                                        new InstantCommand(() -> robot.claw.releaseLeft()),
                                        new WaitCommand(350),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement4Middle),
                                                new SecondOuttakeCommand(robot)
                                        ),
                                        new WaitCommand(600),
                                        new InstantCommand(() -> robot.claw.releaseRight()),
                                        new ParallelCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement5Middle),
                                                new RestCommand(robot)
                                        )

                                )
                        );

                        break;
                }
            }
        } else if (!plusOne) {
            if (truss) { //REGULAR TRUSS
                if (parkLeft) {
                    switch (recordedPropPosition) {
                        case RIGHT:
                            TrajectorySequence movement1Right = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, 63.48, Math.toRadians(-90.00)))
                                    .splineToConstantHeading(
                                            new Vector2d(-51.6, 43.70), Math.toRadians(-90.00),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToSplineHeading(
                                            new Pose2d(-46, 60.5, Math.toRadians(0.00)),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(9, 60.5),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .turn(Math.toRadians(-30))
                                    .build();

                            TrajectorySequence movement2Right = robot.driveSubsystem.trajectorySequenceBuilder(movement1Right.end())
                                    .lineToSplineHeading(
                                            new Pose2d(37.72, 60.5, Math.toRadians(0.00)),
                                            SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(37.72, 32.5),
                                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(48, 32.5),
                                            SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();

                            TrajectorySequence movement3Right = robot.driveSubsystem.trajectorySequenceBuilder(movement2Right.end())
                                    .lineToConstantHeading(
                                            new Vector2d(33, 30),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(33, 61),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(52, 61),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();

                            CommandScheduler.getInstance().schedule(
                                    new SequentialCommandGroup(
                                            new DriveCommand(robot.driveSubsystem, movement1Right),
                                            new WaitUntilCommand(() -> robotSensedWall || time_since_start.seconds() > 22),
                                            new ParallelCommandGroup(
                                                    new LowOuttakeCommand(robot),
                                                    new DriveCommand(robot.driveSubsystem, movement2Right)
                                            ),
                                            new WaitCommand(350),
                                            new InstantCommand(() -> robot.claw.releaseLeft()),
                                            new WaitCommand(350),
                                            new ParallelCommandGroup(
                                                    new DriveCommand(robot.driveSubsystem, movement3Right),
                                                    new RestCommand(robot)
                                            )

                                    )
                            );
                            break;
                        case LEFT:
                        case UNFOUND:
                            TrajectorySequence movement1Left = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, 63.48, Math.toRadians(-90.00)))
                                    .splineTo(
                                            new Vector2d(-34.99, 37.93), Math.toRadians(-45.00),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToSplineHeading(
                                            new Pose2d(-46, 60.5, Math.toRadians(0.00)),
                                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(9, 60.5),
                                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .turn(Math.toRadians(-30))
                                    .build();

                            TrajectorySequence movement2Left = robot.driveSubsystem.trajectorySequenceBuilder(movement1Left.end())
                                    .lineToSplineHeading(
                                            new Pose2d(37.72, 59, Math.toRadians(0.00)),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(37.72, 41.2),
                                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(49.5, 41.2),
                                            SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();

                            TrajectorySequence movement3Left = robot.driveSubsystem.trajectorySequenceBuilder(movement2Left.end())
                                    .lineToConstantHeading(
                                            new Vector2d(33, 39.5),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(33, 61),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(52, 61),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();

                            CommandScheduler.getInstance().schedule(
                                    new SequentialCommandGroup(
                                            new DriveCommand(robot.driveSubsystem, movement1Left),
                                            new WaitUntilCommand(() -> robotSensedWall || time_since_start.seconds() > 22),
                                            new ParallelCommandGroup(
                                                    new LowOuttakeCommand(robot),
                                                    new DriveCommand(robot.driveSubsystem, movement2Left)
                                            ),
                                            new InstantCommand(() -> robot.claw.releaseLeft()),
                                            new WaitCommand(350),
                                            new ParallelCommandGroup(
                                                    new DriveCommand(robot.driveSubsystem, movement3Left),
                                                    new RestCommand(robot)
                                            )

                                    )
                            );
                            break;
                        case MIDDLE:
                            TrajectorySequence movement1Middle = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, 63.48, Math.toRadians(-90.00)))
                                    .splineToConstantHeading(
                                            new Vector2d(-45.5, 33.00), Math.toRadians(-90.00),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToSplineHeading(
                                            new Pose2d(-46, 60, Math.toRadians(0.00)),
                                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(9, 60),
                                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .turn(Math.toRadians(-30))
                                    .build();

                            TrajectorySequence movement2Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement1Middle.end())
                                    .lineToSplineHeading(
                                            new Pose2d(37.72, 59, Math.toRadians(0.00)),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(37.72, 36.5),
                                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(50, 36.5),
                                            SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();

                            TrajectorySequence movement3Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement2Middle.end())
                                    .lineToConstantHeading(
                                            new Vector2d(33, 36.5),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(33, 61),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(52, 61),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();

                            CommandScheduler.getInstance().schedule(
                                    new SequentialCommandGroup(
                                            new DriveCommand(robot.driveSubsystem, movement1Middle),
                                            new WaitUntilCommand(() -> robotSensedWall || time_since_start.seconds() > 22),
                                            new ParallelCommandGroup(
                                                    new LowOuttakeCommand(robot),
                                                    new DriveCommand(robot.driveSubsystem, movement2Middle)
                                            ),
                                            new WaitCommand(350),
                                            new InstantCommand(() -> robot.claw.releaseLeft()),
                                            new WaitCommand(350),
                                            new ParallelCommandGroup(
                                                    new DriveCommand(robot.driveSubsystem, movement3Middle),
                                                    new RestCommand(robot)
                                            )

                                    )
                            );
                            break;
                    }
                }
                if (parkRight) {
                    switch (recordedPropPosition) {
                        case RIGHT:
                            TrajectorySequence movement1Right = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, 63.48, Math.toRadians(-90.00)))
                                    .splineToConstantHeading(
                                            new Vector2d(-51.6, 43.70), Math.toRadians(-90.00),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToSplineHeading(
                                            new Pose2d(-46, 60.5, Math.toRadians(0.00)),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(9, 60.5),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .turn(Math.toRadians(-30))
                                    .build();

                            TrajectorySequence movement2Right = robot.driveSubsystem.trajectorySequenceBuilder(movement1Right.end())
                                    .lineToSplineHeading(
                                            new Pose2d(37.72, 60.5, Math.toRadians(0.00)),
                                            SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(37.72, 32.5),
                                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(48, 32.5),
                                            SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();

                            TrajectorySequence movement3Right = robot.driveSubsystem.trajectorySequenceBuilder(movement2Right.end())
                                    .lineToConstantHeading(
                                            new Vector2d(33, 30),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(33, 12),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(52, 12),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();

                            CommandScheduler.getInstance().schedule(
                                    new SequentialCommandGroup(
                                            new DriveCommand(robot.driveSubsystem, movement1Right),
                                            new WaitUntilCommand(() -> robotSensedWall || time_since_start.seconds() > 22),
                                            new ParallelCommandGroup(
                                                    new LowOuttakeCommand(robot),
                                                    new DriveCommand(robot.driveSubsystem, movement2Right)
                                            ),
                                            new WaitCommand(350),
                                            new InstantCommand(() -> robot.claw.releaseLeft()),
                                            new WaitCommand(350),
                                            new ParallelCommandGroup(
                                                    new DriveCommand(robot.driveSubsystem, movement3Right),
                                                    new RestCommand(robot)
                                            )

                                    )
                            );
                            break;
                        case LEFT:
                        case UNFOUND:
                            TrajectorySequence movement1Left = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, 63.48, Math.toRadians(-90.00)))
                                    .splineTo(
                                            new Vector2d(-34.99, 37.93), Math.toRadians(-45.00),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToSplineHeading(
                                            new Pose2d(-46, 60.5, Math.toRadians(0.00)),
                                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(9, 60.5),
                                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .turn(Math.toRadians(-30))
                                    .build();

                            TrajectorySequence movement2Left = robot.driveSubsystem.trajectorySequenceBuilder(movement1Left.end())
                                    .lineToSplineHeading(
                                            new Pose2d(37.72, 59, Math.toRadians(0.00)),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(37.72, 41.2),
                                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(49.5, 41.2),
                                            SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();

                            TrajectorySequence movement3Left = robot.driveSubsystem.trajectorySequenceBuilder(movement2Left.end())
                                    .lineToConstantHeading(
                                            new Vector2d(33, 39.5),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(33, 10),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(52, 10),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();

                            CommandScheduler.getInstance().schedule(
                                    new SequentialCommandGroup(
                                            new DriveCommand(robot.driveSubsystem, movement1Left),
                                            new WaitUntilCommand(() -> robotSensedWall || time_since_start.seconds() > 22),
                                            new ParallelCommandGroup(
                                                    new LowOuttakeCommand(robot),
                                                    new DriveCommand(robot.driveSubsystem, movement2Left)
                                            ),
                                            new InstantCommand(() -> robot.claw.releaseLeft()),
                                            new WaitCommand(350),
                                            new ParallelCommandGroup(
                                                    new DriveCommand(robot.driveSubsystem, movement3Left),
                                                    new RestCommand(robot)
                                            )

                                    )
                            );
                            break;
                        case MIDDLE:
                            TrajectorySequence movement1Middle = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, 63.48, Math.toRadians(-90.00)))
                                    .splineToConstantHeading(
                                            new Vector2d(-45.5, 33.00), Math.toRadians(-90.00),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToSplineHeading(
                                            new Pose2d(-46, 60, Math.toRadians(0.00)),
                                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(9, 60),
                                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .turn(Math.toRadians(-30))
                                    .build();

                            TrajectorySequence movement2Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement1Middle.end())
                                    .lineToSplineHeading(
                                            new Pose2d(37.72, 59, Math.toRadians(0.00)),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(37.72, 36.5),
                                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(50, 36.5),
                                            SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();

                            TrajectorySequence movement3Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement2Middle.end())
                                    .lineToConstantHeading(
                                            new Vector2d(33, 36.5),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(33, 10),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(52, 10),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();

                            CommandScheduler.getInstance().schedule(
                                    new SequentialCommandGroup(
                                            new DriveCommand(robot.driveSubsystem, movement1Middle),
                                            new WaitUntilCommand(() -> robotSensedWall || time_since_start.seconds() > 22),
                                            new ParallelCommandGroup(
                                                    new LowOuttakeCommand(robot),
                                                    new DriveCommand(robot.driveSubsystem, movement2Middle)
                                            ),
                                            new WaitCommand(350),
                                            new InstantCommand(() -> robot.claw.releaseLeft()),
                                            new WaitCommand(350),
                                            new ParallelCommandGroup(
                                                    new DriveCommand(robot.driveSubsystem, movement3Middle),
                                                    new RestCommand(robot)
                                            )

                                    )
                            );
                            break;
                    }
                    if (parkBoard) {
                        switch (recordedPropPosition) {
                            case RIGHT:
                                TrajectorySequence movement1Right = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, 63.48, Math.toRadians(-90.00)))
                                        .splineToConstantHeading(
                                                new Vector2d(-51.6, 43.70), Math.toRadians(-90.00),
                                                SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                        )
                                        .lineToSplineHeading(
                                                new Pose2d(-46, 60.5, Math.toRadians(0.00)),
                                                SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                        )
                                        .lineToConstantHeading(
                                                new Vector2d(9, 60.5),
                                                SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                        )
                                        .turn(Math.toRadians(-30))
                                        .build();

                                TrajectorySequence movement2Right = robot.driveSubsystem.trajectorySequenceBuilder(movement1Right.end())
                                        .lineToSplineHeading(
                                                new Pose2d(37.72, 60.5, Math.toRadians(0.00)),
                                                SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                        )
                                        .lineToConstantHeading(
                                                new Vector2d(37.72, 32.5),
                                                SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                        )
                                        .lineToConstantHeading(
                                                new Vector2d(48, 32.5),
                                                SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                        )
                                        .build();

                                TrajectorySequence movement3Right = robot.driveSubsystem.trajectorySequenceBuilder(movement2Right.end())
                                        .back(6)
                                        .build();

                                CommandScheduler.getInstance().schedule(
                                        new SequentialCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement1Right),
                                                new WaitUntilCommand(() -> robotSensedWall || time_since_start.seconds() > 22),
                                                new ParallelCommandGroup(
                                                        new LowOuttakeCommand(robot),
                                                        new DriveCommand(robot.driveSubsystem, movement2Right)
                                                ),
                                                new WaitCommand(350),
                                                new InstantCommand(() -> robot.claw.releaseLeft()),
                                                new WaitCommand(350),
                                                new ParallelCommandGroup(
                                                        new DriveCommand(robot.driveSubsystem, movement3Right),
                                                        new RestCommand(robot)
                                                )

                                        )
                                );
                                break;
                            case LEFT:
                            case UNFOUND:
                                TrajectorySequence movement1Left = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, 63.48, Math.toRadians(-90.00)))
                                        .splineTo(
                                                new Vector2d(-34.99, 37.93), Math.toRadians(-45.00),
                                                SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                        )
                                        .lineToSplineHeading(
                                                new Pose2d(-46, 60.5, Math.toRadians(0.00)),
                                                SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                        )
                                        .lineToConstantHeading(
                                                new Vector2d(9, 60.5),
                                                SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                        )
                                        .turn(Math.toRadians(-30))
                                        .build();

                                TrajectorySequence movement2Left = robot.driveSubsystem.trajectorySequenceBuilder(movement1Left.end())
                                        .lineToSplineHeading(
                                                new Pose2d(37.72, 59, Math.toRadians(0.00)),
                                                SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                        )
                                        .lineToConstantHeading(
                                                new Vector2d(37.72, 41.2),
                                                SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                        )
                                        .lineToConstantHeading(
                                                new Vector2d(49.5, 41.2),
                                                SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                        )
                                        .build();

                                TrajectorySequence movement3Left = robot.driveSubsystem.trajectorySequenceBuilder(movement2Left.end())
                                        .back(6)
                                        .build();

                                CommandScheduler.getInstance().schedule(
                                        new SequentialCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement1Left),
                                                new WaitUntilCommand(() -> robotSensedWall || time_since_start.seconds() > 22),
                                                new ParallelCommandGroup(
                                                        new LowOuttakeCommand(robot),
                                                        new DriveCommand(robot.driveSubsystem, movement2Left)
                                                ),
                                                new InstantCommand(() -> robot.claw.releaseLeft()),
                                                new WaitCommand(350),
                                                new ParallelCommandGroup(
                                                        new DriveCommand(robot.driveSubsystem, movement3Left),
                                                        new RestCommand(robot)
                                                )

                                        )
                                );
                                break;
                            case MIDDLE:
                                TrajectorySequence movement1Middle = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, 63.48, Math.toRadians(-90.00)))
                                        .splineToConstantHeading(
                                                new Vector2d(-45.5, 33.00), Math.toRadians(-90.00),
                                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                        )
                                        .lineToSplineHeading(
                                                new Pose2d(-46, 60, Math.toRadians(0.00)),
                                                SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                        )
                                        .lineToConstantHeading(
                                                new Vector2d(9, 60),
                                                SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                        )
                                        .turn(Math.toRadians(-30))
                                        .build();

                                TrajectorySequence movement2Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement1Middle.end())
                                        .lineToSplineHeading(
                                                new Pose2d(37.72, 59, Math.toRadians(0.00)),
                                                SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                        )
                                        .lineToConstantHeading(
                                                new Vector2d(37.72, 36.5),
                                                SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                        )
                                        .lineToConstantHeading(
                                                new Vector2d(50, 36.5),
                                                SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                        )
                                        .build();

                                TrajectorySequence movement3Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement2Middle.end())
                                        .back(6)
                                        .build();

                                CommandScheduler.getInstance().schedule(
                                        new SequentialCommandGroup(
                                                new DriveCommand(robot.driveSubsystem, movement1Middle),
                                                new WaitUntilCommand(() -> robotSensedWall || time_since_start.seconds() > 22),
                                                new ParallelCommandGroup(
                                                        new LowOuttakeCommand(robot),
                                                        new DriveCommand(robot.driveSubsystem, movement2Middle)
                                                ),
                                                new WaitCommand(350),
                                                new InstantCommand(() -> robot.claw.releaseLeft()),
                                                new WaitCommand(350),
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
            } else if (center) { //REGULAR CENTER
                if (parkLeft) {
                    switch (recordedPropPosition) {
                        case LEFT:
                        case UNFOUND:
                            TrajectorySequence movement1Left = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, 63.48, Math.toRadians(-90.00)))
                                    .splineToLinearHeading(new Pose2d(-34.99, 37.93, Math.toRadians(-45)), Math.toRadians(-45.00))
                                    .lineToSplineHeading(new Pose2d(-46.84, 43.28, Math.toRadians(270.00)))
                                    .lineToConstantHeading(new Vector2d(-46.54, 14))
                                    .lineToSplineHeading(new Pose2d(8, 14, Math.toRadians(30.00)))
                                    .build();

                            TrajectorySequence movement2Left = robot.driveSubsystem.trajectorySequenceBuilder(movement1Left.end())
                                    .lineToSplineHeading(new Pose2d(37.00, 14, Math.toRadians(0.00)))
                                    .lineToConstantHeading(new Vector2d(37.00, 44.5))
                                    .lineToConstantHeading(new Vector2d(46, 44.5))
                                    .build();

                            TrajectorySequence movement3Left = robot.driveSubsystem.trajectorySequenceBuilder(movement2Left.end())
                                    .lineToConstantHeading(
                                            new Vector2d(35, 44.5),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(35, 66),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(54, 66),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();

                            CommandScheduler.getInstance().schedule(
                                    new SequentialCommandGroup(
                                            new DriveCommand(robot.driveSubsystem, movement1Left),
                                            new WaitCommand(350),
                                            new WaitUntilCommand(() -> robotSensed || time_since_start.seconds() > 19),
                                            new ParallelCommandGroup(
                                                    new DriveCommand(robot.driveSubsystem, movement2Left),
                                                    new LowOuttakeCommand(robot)
                                            ),
                                            new WaitCommand(750),
                                            new InstantCommand(() -> robot.claw.releaseLeft()),
                                            new WaitCommand(400),
                                            new ParallelCommandGroup(
                                                    new RestCommand(robot),
                                                    new DriveCommand(robot.driveSubsystem, movement3Left)
                                            )

                                    )
                            );
                            break;
                        case RIGHT:
                            TrajectorySequence movement1Right = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, 63.48, Math.toRadians(-90.00)))
                                    .splineToConstantHeading(
                                            new Vector2d(-52.39, 41.70), Math.toRadians(270.00),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(-52.39, 50.41),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(-36.54, 50.41),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(-36.54, 14),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToSplineHeading(new Pose2d(8, 14, Math.toRadians(30.00)))
                                    .build();

                            TrajectorySequence movement2Right = robot.driveSubsystem.trajectorySequenceBuilder(movement1Right.end())
                                    .lineToSplineHeading(new Pose2d(37.54, 14, Math.toRadians(0.00)))
                                    .lineToConstantHeading(
                                            new Vector2d(37.54, 32.5),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(48, 32.5),
                                            SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();

                            TrajectorySequence movement3Right = robot.driveSubsystem.trajectorySequenceBuilder(movement2Right.end())
                                    .lineToConstantHeading(
                                            new Vector2d(36.91, 30),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(36.52, 66),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(54.73, 66),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();

                            CommandScheduler.getInstance().schedule(
                                    new SequentialCommandGroup(
                                            new DriveCommand(robot.driveSubsystem, movement1Right),
                                            new WaitCommand(350),
                                            new WaitUntilCommand(() -> robotSensed || time_since_start.seconds() > 19),
                                            new ParallelCommandGroup(
                                                    new DriveCommand(robot.driveSubsystem, movement2Right),
                                                    new LowOuttakeCommand(robot)
                                            ),
                                            new WaitCommand(750),
                                            new InstantCommand(() -> robot.claw.releaseLeft()),
                                            new WaitCommand(2000),
                                            new ParallelCommandGroup(
                                                    new DriveCommand(robot.driveSubsystem, movement3Right),
                                                    new RestCommand(robot)
                                            )

                                    )
                            );
                            break;
                        case MIDDLE:
                            TrajectorySequence movement1Middle = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, 63.48, Math.toRadians(-90.00)))
                                    .splineToConstantHeading(
                                            new Vector2d(-44.50, 34.5), Math.toRadians(270.00),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(-43.50, 45.46),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(-52.39, 45.66),
                                            SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(-53.78, 14),
                                            SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToSplineHeading(new Pose2d(8, 14, Math.toRadians(30.00)))
                                    .build();

                            TrajectorySequence movement2Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement1Middle.end())
                                    .lineToSplineHeading(
                                            new Pose2d(37.34, 14, Math.toRadians(0.00)),
                                            SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(37.54, 39.5),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(48.5, 39.5),
                                            SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();

                            TrajectorySequence movement3Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement2Middle.end())
                                    .lineToConstantHeading(
                                            new Vector2d(38, 39),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(39.52, 66),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(54.73, 66),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();

                            CommandScheduler.getInstance().schedule(
                                    new SequentialCommandGroup(
                                            new DriveCommand(robot.driveSubsystem, movement1Middle),
                                            new WaitCommand(350),
                                            new WaitUntilCommand(() -> robotSensed || time_since_start.seconds() > 19),
                                            new ParallelCommandGroup(
                                                    new DriveCommand(robot.driveSubsystem, movement2Middle),
                                                    new LowOuttakeCommand(robot)
                                            ),
                                            new WaitCommand(750),
                                            new InstantCommand(() -> robot.claw.releaseLeft()),
                                            new WaitCommand(500),
                                            new ParallelCommandGroup(
                                                    new DriveCommand(robot.driveSubsystem, movement3Middle),
                                                    new RestCommand(robot)
                                            )

                                    )
                            );
                            break;
                    }
                }
                if (parkRight) {
                    switch (recordedPropPosition) {
                        case LEFT:
                        case UNFOUND:
                            TrajectorySequence movement1Left = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, 63.48, Math.toRadians(-90.00)))
                                    .splineToLinearHeading(new Pose2d(-34.99, 37.93, Math.toRadians(-45)), Math.toRadians(-45.00))
                                    .lineToSplineHeading(new Pose2d(-46.84, 43.28, Math.toRadians(270.00)))
                                    .lineToConstantHeading(new Vector2d(-46.54, 14))
                                    .lineToSplineHeading(new Pose2d(8, 14, Math.toRadians(30.00)))
                                    .build();

                            TrajectorySequence movement2Left = robot.driveSubsystem.trajectorySequenceBuilder(movement1Left.end())
                                    .lineToSplineHeading(new Pose2d(37.00, 14, Math.toRadians(0.00)))
                                    .lineToConstantHeading(new Vector2d(37.00, 44.5))
                                    .lineToConstantHeading(new Vector2d(46, 44.5))
                                    .build();

                            TrajectorySequence movement3Left = robot.driveSubsystem.trajectorySequenceBuilder(movement2Left.end())
                                    .lineToConstantHeading(
                                            new Vector2d(35, 44.5),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(35, 13.5),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(54, 13.5),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();

                            CommandScheduler.getInstance().schedule(
                                    new SequentialCommandGroup(
                                            new DriveCommand(robot.driveSubsystem, movement1Left),
                                            new WaitCommand(350),
                                            new WaitUntilCommand(() -> robotSensed || time_since_start.seconds() > 19),
                                            new ParallelCommandGroup(
                                                    new DriveCommand(robot.driveSubsystem, movement2Left),
                                                    new LowOuttakeCommand(robot)
                                            ),
                                            new WaitCommand(750),
                                            new InstantCommand(() -> robot.claw.releaseLeft()),
                                            new WaitCommand(400),
                                            new ParallelCommandGroup(
                                                    new RestCommand(robot),
                                                    new DriveCommand(robot.driveSubsystem, movement3Left)
                                            )

                                    )
                            );
                            break;
                        case RIGHT:
                            TrajectorySequence movement1Right = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, 63.48, Math.toRadians(-90.00)))
                                    .splineToConstantHeading(
                                            new Vector2d(-52.39, 41.70), Math.toRadians(270.00),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(-52.39, 50.41),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(-36.54, 50.41),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(-36.54, 14),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToSplineHeading(new Pose2d(8, 14, Math.toRadians(30.00)))
                                    .build();

                            TrajectorySequence movement2Right = robot.driveSubsystem.trajectorySequenceBuilder(movement1Right.end())
                                    .lineToSplineHeading(new Pose2d(37.54, 14, Math.toRadians(0.00)))
                                    .lineToConstantHeading(
                                            new Vector2d(37.54, 32.5),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(48, 32.5),
                                            SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();

                            TrajectorySequence movement3Right = robot.driveSubsystem.trajectorySequenceBuilder(movement2Right.end())
                                    .lineToConstantHeading(
                                            new Vector2d(36.91, 30),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(36.52, 13.5),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(54.73, 13.5),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();

                            CommandScheduler.getInstance().schedule(
                                    new SequentialCommandGroup(
                                            new DriveCommand(robot.driveSubsystem, movement1Right),
                                            new WaitCommand(350),
                                            new WaitUntilCommand(() -> robotSensed || time_since_start.seconds() > 19),
                                            new ParallelCommandGroup(
                                                    new DriveCommand(robot.driveSubsystem, movement2Right),
                                                    new LowOuttakeCommand(robot)
                                            ),
                                            new WaitCommand(750),
                                            new InstantCommand(() -> robot.claw.releaseLeft()),
                                            new WaitCommand(2000),
                                            new ParallelCommandGroup(
                                                    new DriveCommand(robot.driveSubsystem, movement3Right),
                                                    new RestCommand(robot)
                                            )

                                    )
                            );
                            break;
                        case MIDDLE:
                            TrajectorySequence movement1Middle = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, 63.48, Math.toRadians(-90.00)))
                                    .splineToConstantHeading(
                                            new Vector2d(-44.50, 34.5), Math.toRadians(270.00),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(-43.50, 45.46),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(-52.39, 45.66),
                                            SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(-53.78, 14),
                                            SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToSplineHeading(new Pose2d(8, 14, Math.toRadians(30.00)))
                                    .build();

                            TrajectorySequence movement2Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement1Middle.end())
                                    .lineToSplineHeading(
                                            new Pose2d(37.34, 14, Math.toRadians(0.00)),
                                            SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(37.54, 39.5),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(48.5, 39.5),
                                            SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();

                            TrajectorySequence movement3Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement2Middle.end())
                                    .lineToConstantHeading(
                                            new Vector2d(38, 39),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(39.52, 13.5),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(54.73, 13.5),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();

                            CommandScheduler.getInstance().schedule(
                                    new SequentialCommandGroup(
                                            new DriveCommand(robot.driveSubsystem, movement1Middle),
                                            new WaitCommand(350),
                                            new WaitUntilCommand(() -> robotSensed || time_since_start.seconds() > 19),
                                            new ParallelCommandGroup(
                                                    new DriveCommand(robot.driveSubsystem, movement2Middle),
                                                    new LowOuttakeCommand(robot)
                                            ),
                                            new WaitCommand(750),
                                            new InstantCommand(() -> robot.claw.releaseLeft()),
                                            new WaitCommand(500),
                                            new ParallelCommandGroup(
                                                    new DriveCommand(robot.driveSubsystem, movement3Middle),
                                                    new RestCommand(robot)
                                            )

                                    )
                            );
                            break;
                    }
                }
                if (parkBoard) {
                    switch (recordedPropPosition) {
                        case LEFT:
                        case UNFOUND:
                            TrajectorySequence movement1Left = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, 63.48, Math.toRadians(-90.00)))
                                    .splineToLinearHeading(new Pose2d(-34.99, 37.93, Math.toRadians(-45)), Math.toRadians(-45.00))
                                    .lineToSplineHeading(new Pose2d(-46.84, 43.28, Math.toRadians(270.00)))
                                    .lineToConstantHeading(new Vector2d(-46.54, 14))
                                    .lineToSplineHeading(new Pose2d(8, 14, Math.toRadians(30.00)))
                                    .build();

                            TrajectorySequence movement2Left = robot.driveSubsystem.trajectorySequenceBuilder(movement1Left.end())
                                    .lineToSplineHeading(new Pose2d(37.00, 14, Math.toRadians(0.00)))
                                    .lineToConstantHeading(new Vector2d(37.00, 44.5))
                                    .lineToConstantHeading(new Vector2d(46, 44.5))
                                    .build();

                            TrajectorySequence movement3Left = robot.driveSubsystem.trajectorySequenceBuilder(movement2Left.end())
                                    .back(6)
                                    .build();

                            CommandScheduler.getInstance().schedule(
                                    new SequentialCommandGroup(
                                            new DriveCommand(robot.driveSubsystem, movement1Left),
                                            new WaitCommand(350),
                                            new WaitUntilCommand(() -> robotSensed || time_since_start.seconds() > 19),
                                            new ParallelCommandGroup(
                                                    new DriveCommand(robot.driveSubsystem, movement2Left),
                                                    new LowOuttakeCommand(robot)
                                            ),
                                            new WaitCommand(750),
                                            new InstantCommand(() -> robot.claw.releaseLeft()),
                                            new WaitCommand(400),
                                            new ParallelCommandGroup(
                                                    new RestCommand(robot),
                                                    new DriveCommand(robot.driveSubsystem, movement3Left)
                                            )

                                    )
                            );
                            break;
                        case RIGHT:
                            TrajectorySequence movement1Right = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, 63.48, Math.toRadians(-90.00)))
                                    .splineToConstantHeading(
                                            new Vector2d(-52.39, 41.70), Math.toRadians(270.00),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(-52.39, 50.41),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(-36.54, 50.41),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(-36.54, 14),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToSplineHeading(new Pose2d(8, 14, Math.toRadians(30.00)))
                                    .build();

                            TrajectorySequence movement2Right = robot.driveSubsystem.trajectorySequenceBuilder(movement1Right.end())
                                    .lineToSplineHeading(new Pose2d(37.54, 14, Math.toRadians(0.00)))
                                    .lineToConstantHeading(
                                            new Vector2d(37.54, 32.5),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(48, 32.5),
                                            SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();

                            TrajectorySequence movement3Right = robot.driveSubsystem.trajectorySequenceBuilder(movement2Right.end())
                                    .back(6)
                                    .build();

                            CommandScheduler.getInstance().schedule(
                                    new SequentialCommandGroup(
                                            new DriveCommand(robot.driveSubsystem, movement1Right),
                                            new WaitCommand(350),
                                            new WaitUntilCommand(() -> robotSensed || time_since_start.seconds() > 19),
                                            new ParallelCommandGroup(
                                                    new DriveCommand(robot.driveSubsystem, movement2Right),
                                                    new LowOuttakeCommand(robot)
                                            ),
                                            new WaitCommand(750),
                                            new InstantCommand(() -> robot.claw.releaseLeft()),
                                            new WaitCommand(2000),
                                            new ParallelCommandGroup(
                                                    new DriveCommand(robot.driveSubsystem, movement3Right),
                                                    new RestCommand(robot)
                                            )

                                    )
                            );
                            break;
                        case MIDDLE:
                            TrajectorySequence movement1Middle = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, 63.48, Math.toRadians(-90.00)))
                                    .splineToConstantHeading(
                                            new Vector2d(-44.50, 34.5), Math.toRadians(270.00),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(-43.50, 45.46),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(-52.39, 45.66),
                                            SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(-53.78, 14),
                                            SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToSplineHeading(new Pose2d(8, 14, Math.toRadians(30.00)))
                                    .build();

                            TrajectorySequence movement2Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement1Middle.end())
                                    .lineToSplineHeading(
                                            new Pose2d(37.34, 14, Math.toRadians(0.00)),
                                            SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(37.54, 39.5),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(48.5, 39.5),
                                            SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();

                            TrajectorySequence movement3Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement2Middle.end())
                                    .back(6)
                                    .build();

                            CommandScheduler.getInstance().schedule(
                                    new SequentialCommandGroup(
                                            new DriveCommand(robot.driveSubsystem, movement1Middle),
                                            new WaitCommand(350),
                                            new WaitUntilCommand(() -> robotSensed || time_since_start.seconds() > 19),
                                            new ParallelCommandGroup(
                                                    new DriveCommand(robot.driveSubsystem, movement2Middle),
                                                    new LowOuttakeCommand(robot)
                                            ),
                                            new WaitCommand(750),
                                            new InstantCommand(() -> robot.claw.releaseLeft()),
                                            new WaitCommand(500),
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

        robotSensedWall = (robotProcessor.getSensedBoolean() != BlueWallRobotScan.Sensed.TRUE);

        robotSensed = (robotProcessor2.getSensedBoolean() != BlueCenterRobotScan.Sensed.TRUE);

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