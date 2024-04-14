package org.firstinspires.ftc.teamcode.Drive.OpModes.Worlds.Red.Left;

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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.TrajectorySequences.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.DriveCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.HighOuttakeCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.LowOuttakeCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.PlusOneRedStackCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.RestCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.SecondOuttakeCommand;
import org.firstinspires.ftc.teamcode.Utility.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.RoadRunner.DriveConstants;
import org.firstinspires.ftc.teamcode.Utility.RoadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Utility.Vision.Prop.NewRedLeftProcessor;
import org.firstinspires.ftc.teamcode.Utility.Vision.Robot.Center.RedCenterRobotScan;
import org.firstinspires.ftc.teamcode.Utility.Vision.Robot.Wall.RedWallRobotScan;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class RedLeft extends OpMode {
    private VisionPortal visionPortal;
    private NewRedLeftProcessor colorMassDetectionProcessor;
    private RedCenterRobotScan robotProcessor;
    private RedWallRobotScan robotProcessor2;
    boolean robotSensed;
    boolean robotSensedWall;

    private RobotHardware robot;
    private ElapsedTime time_since_start;
    private double loop;
    boolean previousX, currentX;
    boolean parkLeft = false, parkBoard = false, parkRight = true;
    boolean previousY, currentY;
    boolean plusOne = false;
    boolean previousB, currentB;
    boolean truss = false, center = true;
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
        robotProcessor = new RedCenterRobotScan();
        robotProcessor2 = new RedWallRobotScan();

        colorMassDetectionProcessor.setDetectionColor(true); //false is blue, true is red
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .addProcessors(colorMassDetectionProcessor, robotProcessor, robotProcessor2)
                .build();
    }

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

        NewRedLeftProcessor.PropPositions recordedPropPosition = colorMassDetectionProcessor.getPropLocation();
        robot.driveSubsystem.setPoseEstimate(new Pose2d(-40.11, -63.48, Math.toRadians(450.00)));

        if(plusOne) { //PLUS ONE CODE
            if (parkRight) {
                switch (recordedPropPosition) {
                    case RIGHT:
                        TrajectorySequence movement1Right = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, -63.48, Math.toRadians(90.00)))
                                .splineToLinearHeading(new Pose2d(-34.5, -37.58, Math.toRadians(45)), Math.toRadians(45))
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(-53, -36.5, Math.toRadians(0.00)), Math.toRadians(180))
                                .build();

                        TrajectorySequence extraBack = robot.driveSubsystem.trajectorySequenceBuilder(movement1Right.end())
                                .setReversed(false)
                                .lineToConstantHeading(
                                        new Vector2d(-59, -36.5),
                                        SampleMecanumDrive.getVelocityConstraint(7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();


                        TrajectorySequence movement2Right = robot.driveSubsystem.trajectorySequenceBuilder(extraBack.end())
                                .lineToConstantHeading(
                                        new Vector2d(-52, -37),
                                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(-52, -58.5),
                                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(6, -58.5),
                                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .turn(Math.toRadians(30))
                                .build();


                        TrajectorySequence movement3Right = robot.driveSubsystem.trajectorySequenceBuilder(movement2Right.end())
                                .lineToSplineHeading(new Pose2d(30, -58.5, Math.toRadians(0.00)))
                                .splineToConstantHeading(
                                        new Vector2d(45, -41.5), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToConstantHeading(
                                        new Vector2d(50, -41.5), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();


                        TrajectorySequence movement4Right = robot.driveSubsystem.trajectorySequenceBuilder(movement3Right.end())
                                .setTangent(-180)
                                .splineToConstantHeading(
                                        new Vector2d(
                                                48.5, -35), Math.toRadians(0),
                                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement5Right = robot.driveSubsystem.trajectorySequenceBuilder(movement4Right.end())
                                .lineToConstantHeading(new Vector2d(43, -30))
                                .lineToConstantHeading(new Vector2d(43, -62))
                                .lineToConstantHeading(new Vector2d(55, -62))
                                .build();

                        CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement1Right),
                                        new PlusOneRedStackCommand(robot),
                                        new DriveCommand(robot.driveSubsystem, extraBack),
                                        new WaitCommand(500),
                                        new InstantCommand(() -> robot.claw.grabBoth()),
                                        new WaitCommand(350),
                                        new DriveCommand(robot.driveSubsystem, movement2Right),
                                        new WaitUntilCommand(() -> robotSensedWall || time_since_start.seconds() > 19),
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
                                .splineToLinearHeading(
                                        new Pose2d(-51.5, -43.70, Math.toRadians(90.00)), Math.toRadians(90.00)
                                )
                                .lineToSplineHeading(
                                        new Pose2d(
                                                -54, -53, Math.toRadians(0.00)),
                                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToSplineHeading(
                                        new Pose2d(-54, -36, Math.toRadians(0.00))
                                )
                                .build();

                        TrajectorySequence extraBack2 = robot.driveSubsystem.trajectorySequenceBuilder(movement1Left.end())
                                .lineToConstantHeading(
                                        new Vector2d(-59, -36),
                                        SampleMecanumDrive.getVelocityConstraint(7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement2Left = robot.driveSubsystem.trajectorySequenceBuilder(extraBack2.end())
                                .lineToConstantHeading(new Vector2d(-56.34, -38))
                                .lineToConstantHeading(new Vector2d(-56.34, -58.5))
                                .lineToConstantHeading(new Vector2d(8, -57))
                                .turn(Math.toRadians(30))
                                .build();

                        TrajectorySequence movement3Left = robot.driveSubsystem.trajectorySequenceBuilder(movement2Left.end())
                                .lineToSplineHeading(new Pose2d(20, -57, Math.toRadians(0.00)))
                                .splineToConstantHeading(
                                        new Vector2d(49.3, -30), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement4Left = robot.driveSubsystem.trajectorySequenceBuilder(movement3Left.end())
                                .setTangent(-180)
                                .splineToConstantHeading(
                                        new Vector2d(
                                                48, -37), Math.toRadians(0),
                                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement5Left = robot.driveSubsystem.trajectorySequenceBuilder(movement4Left.end())
                                .lineToConstantHeading(new Vector2d(43, -37))
                                .lineToConstantHeading(new Vector2d(43, -62))
                                .lineToConstantHeading(new Vector2d(48, -62))
                                .build();

                        CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement1Left),
                                        new PlusOneRedStackCommand(robot),
                                        new WaitCommand(500),
                                        new DriveCommand(robot.driveSubsystem, extraBack2),
                                        new WaitCommand(500),
                                        new InstantCommand(() -> robot.claw.grabBoth()),
                                        new WaitCommand(500),
                                        new DriveCommand(robot.driveSubsystem, movement2Left),
                                        new WaitUntilCommand(() -> robotSensedWall || time_since_start.seconds() > 19),
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
                                        new Vector2d(-44.50, -34.5), Math.toRadians(90.00)
                                )
                                .lineToSplineHeading(
                                        new Pose2d(-55, -37, Math.toRadians(0))
                                )
                                .build();


                        TrajectorySequence extraBack3 = robot.driveSubsystem.trajectorySequenceBuilder(movement1Middle.end())
                                .lineToConstantHeading(
                                        new Vector2d(-59, -37),
                                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();


                        TrajectorySequence movement2Middle = robot.driveSubsystem.trajectorySequenceBuilder(extraBack3.end())
                                .lineToConstantHeading(new Vector2d(-54, -37.5))
                                .lineToConstantHeading(new Vector2d(-54, -58.5))
                                .lineToConstantHeading(new Vector2d(10, -57))
                                .turn(Math.toRadians(30))
                                .build();


                        TrajectorySequence movement3Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement2Middle.end())
                                .lineToSplineHeading(new Pose2d(20, -57, Math.toRadians(0)))
                                .splineToConstantHeading(
                                        new Vector2d(52, -35), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();


                        TrajectorySequence movement4Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement3Middle.end())
                                .setTangent(-180)
                                .splineToConstantHeading(
                                        new Vector2d(
                                                48.9, -40), Math.toRadians(0),
                                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();


                        TrajectorySequence movement5Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement4Middle.end())
                                .lineToConstantHeading(new Vector2d(43.9, -30))
                                .lineToConstantHeading(new Vector2d(43.5, -62))
                                .lineToConstantHeading(new Vector2d(48.5, -62))
                                .build();


                        CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement1Middle),
                                        new WaitCommand(500),
                                        new PlusOneRedStackCommand(robot),
                                        new DriveCommand(robot.driveSubsystem, extraBack3),
                                        new WaitCommand(500),
                                        new InstantCommand(() -> robot.claw.grabBoth()),
                                        new WaitCommand(500),
                                        new DriveCommand(robot.driveSubsystem, movement2Middle),
                                        new WaitUntilCommand(() -> robotSensedWall || time_since_start.seconds() > 19),
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
            } else if (parkLeft) {
                switch (recordedPropPosition) {
                    case RIGHT:
                        TrajectorySequence movement1Right = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, -63.48, Math.toRadians(90.00)))
                                .splineToLinearHeading(new Pose2d(-34.5, -37.58, Math.toRadians(45)), Math.toRadians(45))
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(-53, -36.5, Math.toRadians(0.00)), Math.toRadians(180))
                                .build();

                        TrajectorySequence extraBack = robot.driveSubsystem.trajectorySequenceBuilder(movement1Right.end())
                                .setReversed(false)
                                .lineToConstantHeading(
                                        new Vector2d(-59, -36.5),
                                        SampleMecanumDrive.getVelocityConstraint(7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();


                        TrajectorySequence movement2Right = robot.driveSubsystem.trajectorySequenceBuilder(extraBack.end())
                                .lineToConstantHeading(
                                        new Vector2d(-52, -37),
                                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(-52, -58.5),
                                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(6, -58.5),
                                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .turn(Math.toRadians(30))
                                .build();


                        TrajectorySequence movement3Right = robot.driveSubsystem.trajectorySequenceBuilder(movement2Right.end())
                                .lineToSplineHeading(new Pose2d(30, -58.5, Math.toRadians(0.00)))
                                .splineToConstantHeading(
                                        new Vector2d(45, -41.5), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToConstantHeading(
                                        new Vector2d(50, -41.5), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();


                        TrajectorySequence movement4Right = robot.driveSubsystem.trajectorySequenceBuilder(movement3Right.end())
                                .setTangent(-180)
                                .splineToConstantHeading(
                                        new Vector2d(
                                                48.5, -35), Math.toRadians(0),
                                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement5Right = robot.driveSubsystem.trajectorySequenceBuilder(movement4Right.end())
                                .lineToConstantHeading(new Vector2d(43, -30))
                                .lineToConstantHeading(new Vector2d(43, -12))
                                .lineToConstantHeading(new Vector2d(55, -12))
                                .build();

                        CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement1Right),
                                        new PlusOneRedStackCommand(robot),
                                        new DriveCommand(robot.driveSubsystem, extraBack),
                                        new WaitCommand(500),
                                        new InstantCommand(() -> robot.claw.grabBoth()),
                                        new WaitCommand(350),
                                        new DriveCommand(robot.driveSubsystem, movement2Right),
                                        new WaitUntilCommand(() -> robotSensed || time_since_start.seconds() > 19),
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
                                .splineToLinearHeading(
                                        new Pose2d(-51.5, -43.70, Math.toRadians(90.00)), Math.toRadians(90.00)
                                )
                                .lineToSplineHeading(
                                        new Pose2d(
                                                -54, -53, Math.toRadians(0.00)),
                                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToSplineHeading(
                                        new Pose2d(-54, -36, Math.toRadians(0.00))
                                )
                                .build();

                        TrajectorySequence extraBack2 = robot.driveSubsystem.trajectorySequenceBuilder(movement1Left.end())
                                .lineToConstantHeading(
                                        new Vector2d(-59, -36),
                                        SampleMecanumDrive.getVelocityConstraint(7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement2Left = robot.driveSubsystem.trajectorySequenceBuilder(extraBack2.end())
                                .lineToConstantHeading(new Vector2d(-56.34, -38))
                                .lineToConstantHeading(new Vector2d(-56.34, -58.5))
                                .lineToConstantHeading(new Vector2d(8, -57))
                                .turn(Math.toRadians(30))
                                .build();

                        TrajectorySequence movement3Left = robot.driveSubsystem.trajectorySequenceBuilder(movement2Left.end())
                                .lineToSplineHeading(new Pose2d(20, -57, Math.toRadians(0.00)))
                                .splineToConstantHeading(
                                        new Vector2d(49.3, -30), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement4Left = robot.driveSubsystem.trajectorySequenceBuilder(movement3Left.end())
                                .setTangent(-180)
                                .splineToConstantHeading(
                                        new Vector2d(
                                                48, -37), Math.toRadians(0),
                                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement5Left = robot.driveSubsystem.trajectorySequenceBuilder(movement4Left.end())
                                .lineToConstantHeading(new Vector2d(43, -37))
                                .lineToConstantHeading(new Vector2d(43, -8))
                                .lineToConstantHeading(new Vector2d(48, -8))
                                .build();

                        CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement1Left),
                                        new PlusOneRedStackCommand(robot),
                                        new WaitCommand(500),
                                        new DriveCommand(robot.driveSubsystem, extraBack2),
                                        new WaitCommand(500),
                                        new InstantCommand(() -> robot.claw.grabBoth()),
                                        new WaitCommand(500),
                                        new DriveCommand(robot.driveSubsystem, movement2Left),
                                        new WaitUntilCommand(() -> robotSensed || time_since_start.seconds() > 19),
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
                                        new Vector2d(-44.50, -34.5), Math.toRadians(90.00)
                                )
                                .lineToSplineHeading(
                                        new Pose2d(-55, -37, Math.toRadians(0))
                                )
                                .build();


                        TrajectorySequence extraBack3 = robot.driveSubsystem.trajectorySequenceBuilder(movement1Middle.end())
                                .lineToConstantHeading(
                                        new Vector2d(-59.3, -37),
                                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();


                        TrajectorySequence movement2Middle = robot.driveSubsystem.trajectorySequenceBuilder(extraBack3.end())
                                .lineToConstantHeading(new Vector2d(-54, -37.5))
                                .lineToConstantHeading(new Vector2d(-54, -58.5))
                                .lineToConstantHeading(new Vector2d(10, -57))
                                .turn(Math.toRadians(30))
                                .build();


                        TrajectorySequence movement3Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement2Middle.end())
                                .lineToSplineHeading(new Pose2d(20, -57, Math.toRadians(0)))
                                .splineToConstantHeading(
                                        new Vector2d(52, -35), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();


                        TrajectorySequence movement4Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement3Middle.end())
                                .setTangent(-180)
                                .splineToConstantHeading(
                                        new Vector2d(
                                                48.9, -40), Math.toRadians(0),
                                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();


                        TrajectorySequence movement5Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement4Middle.end())
                                .lineToConstantHeading(new Vector2d(43.9, -30))
                                .lineToConstantHeading(new Vector2d(43.5, -12))
                                .lineToConstantHeading(new Vector2d(48.5, -12))
                                .build();


                        CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement1Middle),
                                        new WaitCommand(500),
                                        new PlusOneRedStackCommand(robot),
                                        new DriveCommand(robot.driveSubsystem, extraBack3),
                                        new WaitCommand(500),
                                        new InstantCommand(() -> robot.claw.grabBoth()),
                                        new WaitCommand(500),
                                        new DriveCommand(robot.driveSubsystem, movement2Middle),
                                        new WaitUntilCommand(() -> robotSensed || time_since_start.seconds() > 19),
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
            } else if (parkBoard) {
                switch (recordedPropPosition) {
                    case RIGHT:
                        TrajectorySequence movement1Right = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, -63.48, Math.toRadians(90.00)))
                                .splineToLinearHeading(new Pose2d(-34.5, -37.58, Math.toRadians(45)), Math.toRadians(45))
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(-53, -36.5, Math.toRadians(0.00)), Math.toRadians(180))
                                .build();

                        TrajectorySequence extraBack = robot.driveSubsystem.trajectorySequenceBuilder(movement1Right.end())
                                .setReversed(false)
                                .lineToConstantHeading(
                                        new Vector2d(-58.25, -36.5),
                                        SampleMecanumDrive.getVelocityConstraint(7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();


                        TrajectorySequence movement2Right = robot.driveSubsystem.trajectorySequenceBuilder(extraBack.end())
                                .lineToConstantHeading(
                                        new Vector2d(-52, -37),
                                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(-52, -58.5),
                                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToConstantHeading(
                                        new Vector2d(6, -58.5),
                                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .turn(Math.toRadians(30))
                                .build();


                        TrajectorySequence movement3Right = robot.driveSubsystem.trajectorySequenceBuilder(movement2Right.end())
                                .lineToSplineHeading(new Pose2d(30, -58.5, Math.toRadians(0.00)))
                                .splineToConstantHeading(
                                        new Vector2d(45, -41.5), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .splineToConstantHeading(
                                        new Vector2d(50, -41.5), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();


                        TrajectorySequence movement4Right = robot.driveSubsystem.trajectorySequenceBuilder(movement3Right.end())
                                .setTangent(-180)
                                .splineToConstantHeading(
                                        new Vector2d(
                                                48.5, -35), Math.toRadians(0),
                                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement5Right = robot.driveSubsystem.trajectorySequenceBuilder(movement4Right.end())
                                .back(6)
                                .build();

                        CommandScheduler.getInstance().schedule(
                                new SequentialCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement1Right),
                                        new PlusOneRedStackCommand(robot),
                                        new DriveCommand(robot.driveSubsystem, extraBack),
                                        new WaitCommand(500),
                                        new InstantCommand(() -> robot.claw.grabBoth()),
                                        new WaitCommand(350),
                                        new DriveCommand(robot.driveSubsystem, movement2Right),
                                        new WaitUntilCommand(() -> robotSensed || time_since_start.seconds() > 19),
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
                                .splineToLinearHeading(
                                        new Pose2d(-51.5, -43.70, Math.toRadians(90.00)), Math.toRadians(90.00)
                                )
                                .lineToSplineHeading(
                                        new Pose2d(
                                                -54, -53, Math.toRadians(0.00)),
                                        SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .lineToSplineHeading(
                                        new Pose2d(-54, -36, Math.toRadians(0.00))
                                )
                                .build();

                        TrajectorySequence extraBack2 = robot.driveSubsystem.trajectorySequenceBuilder(movement1Left.end())
                                .lineToConstantHeading(
                                        new Vector2d(-58, -36),
                                        SampleMecanumDrive.getVelocityConstraint(7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement2Left = robot.driveSubsystem.trajectorySequenceBuilder(extraBack2.end())
                                .lineToConstantHeading(new Vector2d(-56.34, -38))
                                .lineToConstantHeading(new Vector2d(-56.34, -58.5))
                                .lineToConstantHeading(new Vector2d(8, -57))
                                .turn(Math.toRadians(30))
                                .build();

                        TrajectorySequence movement3Left = robot.driveSubsystem.trajectorySequenceBuilder(movement2Left.end())
                                .lineToSplineHeading(new Pose2d(20, -57, Math.toRadians(0.00)))
                                .splineToConstantHeading(
                                        new Vector2d(49.3, -30), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();

                        TrajectorySequence movement4Left = robot.driveSubsystem.trajectorySequenceBuilder(movement3Left.end())
                                .setTangent(-180)
                                .splineToConstantHeading(
                                        new Vector2d(
                                                48, -37), Math.toRadians(0),
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
                                        new PlusOneRedStackCommand(robot),
                                        new WaitCommand(500),
                                        new DriveCommand(robot.driveSubsystem, extraBack2),
                                        new WaitCommand(500),
                                        new InstantCommand(() -> robot.claw.grabBoth()),
                                        new WaitCommand(500),
                                        new DriveCommand(robot.driveSubsystem, movement2Left),
                                        new WaitUntilCommand(() -> robotSensed || time_since_start.seconds() > 19),
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
                                        new Vector2d(-44.50, -34.5), Math.toRadians(90.00)
                                )
                                .lineToSplineHeading(
                                        new Pose2d(-55, -37, Math.toRadians(0))
                                )
                                .build();


                        TrajectorySequence extraBack3 = robot.driveSubsystem.trajectorySequenceBuilder(movement1Middle.end())
                                .lineToConstantHeading(
                                        new Vector2d(-59, -37),
                                        SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();


                        TrajectorySequence movement2Middle = robot.driveSubsystem.trajectorySequenceBuilder(extraBack3.end())
                                .lineToConstantHeading(new Vector2d(-54, -37.5))
                                .lineToConstantHeading(new Vector2d(-54, -58.5))
                                .lineToConstantHeading(new Vector2d(10, -57))
                                .turn(Math.toRadians(30))
                                .build();


                        TrajectorySequence movement3Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement2Middle.end())
                                .lineToSplineHeading(new Pose2d(20, -57, Math.toRadians(0)))
                                .splineToConstantHeading(
                                        new Vector2d(52, -35), Math.toRadians(0.00),
                                        SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                )
                                .build();


                        TrajectorySequence movement4Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement3Middle.end())
                                .setTangent(-180)
                                .splineToConstantHeading(
                                        new Vector2d(
                                                48.9, -40), Math.toRadians(0),
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
                                        new WaitCommand(500),
                                        new PlusOneRedStackCommand(robot),
                                        new DriveCommand(robot.driveSubsystem, extraBack3),
                                        new WaitCommand(500),
                                        new InstantCommand(() -> robot.claw.grabBoth()),
                                        new WaitCommand(500),
                                        new DriveCommand(robot.driveSubsystem, movement2Middle),
                                        new WaitUntilCommand(() -> robotSensed || time_since_start.seconds() > 19),
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
        } else if(!plusOne) {
            if(truss) {
                if (parkRight) {
                    switch (recordedPropPosition) {
                        case LEFT:
                            TrajectorySequence movement1Left = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, -63.48, Math.toRadians(450.00)))
                                    .splineToConstantHeading(
                                            new Vector2d(-51.6, -43.70), Math.toRadians(90.00),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToSplineHeading(
                                            new Pose2d(-46, -59, Math.toRadians(0.00)),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(8, -59),
                                            SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .turn(Math.toRadians(30))
                                    .build();

                            TrajectorySequence movement2Left = robot.driveSubsystem.trajectorySequenceBuilder(movement1Left.end())
                                    .lineToSplineHeading(
                                            new Pose2d(37.72, -59, Math.toRadians(0)),
                                            SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(37.72, -29),
                                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(50, -29),
                                            SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();

                            TrajectorySequence movement3Left = robot.driveSubsystem.trajectorySequenceBuilder(movement2Left.end())
                                    .lineToConstantHeading(
                                            new Vector2d(35, -29),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(35, -62),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(50, -62),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();


                            CommandScheduler.getInstance().schedule(
                                    new SequentialCommandGroup(
                                            new DriveCommand(robot.driveSubsystem, movement1Left),
                                            new WaitUntilCommand(() -> robotSensed || time_since_start.seconds() > 22),
                                            new ParallelCommandGroup(
                                                    new LowOuttakeCommand(robot),
                                                    new DriveCommand(robot.driveSubsystem, movement2Left)
                                            ),
                                            new WaitCommand(350),
                                            new InstantCommand(() -> robot.claw.releaseRight()),
                                            new WaitCommand(350),
                                            new ParallelCommandGroup(
                                                    new DriveCommand(robot.driveSubsystem, movement3Left),
                                                    new RestCommand(robot)
                                            )
                                    )
                            );
                            break;
                        case RIGHT:
                        case UNFOUND:
                            TrajectorySequence movement1Right = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, -63.48, Math.toRadians(450.00)))
                                    .splineTo(
                                            new Vector2d(-34.99, -37.93), Math.toRadians(45.00),
                                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToSplineHeading(
                                            new Pose2d(-46, -57, Math.toRadians(0.00)),
                                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(8, -57),
                                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .turn(Math.toRadians(30))
                                    .build();

                            TrajectorySequence movement2Right = robot.driveSubsystem.trajectorySequenceBuilder(movement1Right.end())
                                    .lineToSplineHeading(
                                            new Pose2d(37.72, -57, Math.toRadians(0.00)),
                                            SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(37.72, -40),
                                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(50, -40),
                                            SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();

                            TrajectorySequence movement3Right = robot.driveSubsystem.trajectorySequenceBuilder(movement2Right.end())
                                    .lineToConstantHeading(
                                            new Vector2d(35, -40),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(35, -62),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(50, -62),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();

                            CommandScheduler.getInstance().schedule(
                                    new SequentialCommandGroup(
                                            new DriveCommand(robot.driveSubsystem, movement1Right),
                                            new WaitUntilCommand(() -> robotSensed || time_since_start.seconds() > 22),
                                            new ParallelCommandGroup(
                                                    new LowOuttakeCommand(robot),
                                                    new DriveCommand(robot.driveSubsystem, movement2Right)
                                            ),
                                            new WaitCommand(350),
                                            new InstantCommand(() -> robot.claw.releaseRight()),
                                            new WaitCommand(350),
                                            new ParallelCommandGroup(
                                                    new DriveCommand(robot.driveSubsystem, movement3Right),
                                                    new RestCommand(robot)
                                            )

                                    )
                            );
                            break;
                        case MIDDLE:
                            TrajectorySequence movement1Middle = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, -63.48, Math.toRadians(450.00)))
                                    .splineToConstantHeading(
                                            new Vector2d(-44.50, -34.5), Math.toRadians(90.00),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToSplineHeading(
                                            new Pose2d(-46, -56, Math.toRadians(0.00)),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(8, -58),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .turn(Math.toRadians(30))
                                    .build();

                            TrajectorySequence movement2Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement1Middle.end())
                                    .lineToSplineHeading(
                                            new Pose2d(37.72, -58, Math.toRadians(0.00)),
                                            SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(37.72, -34),
                                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(51, -34),
                                            SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();

                            TrajectorySequence movement3Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement2Middle.end())
                                    .lineToConstantHeading(
                                            new Vector2d(35, -34),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(35, -62),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(50, -62),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();

                            CommandScheduler.getInstance().schedule(
                                    new SequentialCommandGroup(
                                            new DriveCommand(robot.driveSubsystem, movement1Middle),
                                            new WaitUntilCommand(() -> robotSensed || time_since_start.seconds() > 22),
                                            new ParallelCommandGroup(
                                                    new LowOuttakeCommand(robot),
                                                    new DriveCommand(robot.driveSubsystem, movement2Middle)
                                            ),
                                            new WaitCommand(350),
                                            new InstantCommand(() -> robot.claw.releaseRight()),
                                            new WaitCommand(350),
                                            new ParallelCommandGroup(
                                                    new DriveCommand(robot.driveSubsystem, movement3Middle),
                                                    new RestCommand(robot)
                                            )

                                    )
                            );
                            break;
                    }
                } else if (parkLeft) {
                    switch (recordedPropPosition) {
                        case LEFT:
                            TrajectorySequence movement1Left = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, -63.48, Math.toRadians(450.00)))
                                    .splineToConstantHeading(
                                            new Vector2d(-51.6, -43.70), Math.toRadians(90.00),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToSplineHeading(
                                            new Pose2d(-46, -59, Math.toRadians(0.00)),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(8, -59),
                                            SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .turn(Math.toRadians(30))
                                    .build();

                            TrajectorySequence movement2Left = robot.driveSubsystem.trajectorySequenceBuilder(movement1Left.end())
                                    .lineToSplineHeading(
                                            new Pose2d(37.72, -59, Math.toRadians(0)),
                                            SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(37.72, -29),
                                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(50, -29),
                                            SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();

                            TrajectorySequence movement3Left = robot.driveSubsystem.trajectorySequenceBuilder(movement2Left.end())
                                    .lineToConstantHeading(
                                            new Vector2d(35, -29),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(35, -12),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(50, -12),
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
                                            new WaitCommand(350),
                                            new InstantCommand(() -> robot.claw.releaseRight()),
                                            new WaitCommand(350),
                                            new ParallelCommandGroup(
                                                    new DriveCommand(robot.driveSubsystem, movement3Left),
                                                    new RestCommand(robot)
                                            )
                                    )
                            );
                            break;
                        case RIGHT:
                        case UNFOUND:
                            TrajectorySequence movement1Right = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, -63.48, Math.toRadians(450.00)))
                                    .splineTo(
                                            new Vector2d(-34.99, -37.93), Math.toRadians(45.00),
                                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToSplineHeading(
                                            new Pose2d(-46, -57, Math.toRadians(0.00)),
                                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(8, -57),
                                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .turn(Math.toRadians(30))
                                    .build();

                            TrajectorySequence movement2Right = robot.driveSubsystem.trajectorySequenceBuilder(movement1Right.end())
                                    .lineToSplineHeading(
                                            new Pose2d(37.72, -57, Math.toRadians(0.00)),
                                            SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(37.72, -40),
                                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(50, -40),
                                            SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();

                            TrajectorySequence movement3Right = robot.driveSubsystem.trajectorySequenceBuilder(movement2Right.end())
                                    .lineToConstantHeading(
                                            new Vector2d(35, -40),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(35, -8),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(50, -8),
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
                                            new InstantCommand(() -> robot.claw.releaseRight()),
                                            new WaitCommand(350),
                                            new ParallelCommandGroup(
                                                    new DriveCommand(robot.driveSubsystem, movement3Right),
                                                    new RestCommand(robot)
                                            )

                                    )
                            );
                            break;
                        case MIDDLE:
                            TrajectorySequence movement1Middle = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, -63.48, Math.toRadians(450.00)))
                                    .splineToConstantHeading(
                                            new Vector2d(-44.50, -34.5), Math.toRadians(90.00),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToSplineHeading(
                                            new Pose2d(-46, -56, Math.toRadians(0.00)),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(8, -58),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .turn(Math.toRadians(30))
                                    .build();

                            TrajectorySequence movement2Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement1Middle.end())
                                    .lineToSplineHeading(
                                            new Pose2d(37.72, -58, Math.toRadians(0.00)),
                                            SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(37.72, -34),
                                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(51, -34),
                                            SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();

                            TrajectorySequence movement3Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement2Middle.end())
                                    .lineToConstantHeading(
                                            new Vector2d(35, -34),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(35, -10),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(50, -10),
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
                                            new InstantCommand(() -> robot.claw.releaseRight()),
                                            new WaitCommand(350),
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
                            TrajectorySequence movement1Left = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, -63.48, Math.toRadians(450.00)))
                                    .splineToConstantHeading(
                                            new Vector2d(-51.6, -43.70), Math.toRadians(90.00),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToSplineHeading(
                                            new Pose2d(-46, -59, Math.toRadians(0.00)),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(8, -59),
                                            SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .turn(Math.toRadians(30))
                                    .build();

                            TrajectorySequence movement2Left = robot.driveSubsystem.trajectorySequenceBuilder(movement1Left.end())
                                    .lineToSplineHeading(
                                            new Pose2d(37.72, -59, Math.toRadians(0)),
                                            SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(37.72, -29),
                                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(50, -29),
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
                                            new WaitUntilCommand(() -> robotSensed || time_since_start.seconds() > 22),
                                            new ParallelCommandGroup(
                                                    new LowOuttakeCommand(robot),
                                                    new DriveCommand(robot.driveSubsystem, movement2Left)
                                            ),
                                            new WaitCommand(350),
                                            new InstantCommand(() -> robot.claw.releaseRight()),
                                            new WaitCommand(350),
                                            new ParallelCommandGroup(
                                                    new DriveCommand(robot.driveSubsystem, movement3Left),
                                                    new RestCommand(robot)
                                            )
                                    )
                            );
                            break;
                        case RIGHT:
                        case UNFOUND:
                            TrajectorySequence movement1Right = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, -63.48, Math.toRadians(450.00)))
                                    .splineTo(
                                            new Vector2d(-34.99, -37.93), Math.toRadians(45.00),
                                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToSplineHeading(
                                            new Pose2d(-46, -57, Math.toRadians(0.00)),
                                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(8, -57),
                                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .turn(Math.toRadians(30))
                                    .build();

                            TrajectorySequence movement2Right = robot.driveSubsystem.trajectorySequenceBuilder(movement1Right.end())
                                    .lineToSplineHeading(
                                            new Pose2d(37.72, -57, Math.toRadians(0.00)),
                                            SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(37.72, -40),
                                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(50, -40),
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
                                            new WaitUntilCommand(() -> robotSensed || time_since_start.seconds() > 22),
                                            new ParallelCommandGroup(
                                                    new LowOuttakeCommand(robot),
                                                    new DriveCommand(robot.driveSubsystem, movement2Right)
                                            ),
                                            new WaitCommand(350),
                                            new InstantCommand(() -> robot.claw.releaseRight()),
                                            new WaitCommand(350),
                                            new ParallelCommandGroup(
                                                    new DriveCommand(robot.driveSubsystem, movement3Right),
                                                    new RestCommand(robot)
                                            )

                                    )
                            );
                            break;
                        case MIDDLE:
                            TrajectorySequence movement1Middle = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, -63.48, Math.toRadians(450.00)))
                                    .splineToConstantHeading(
                                            new Vector2d(-44.50, -34.5), Math.toRadians(90.00),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToSplineHeading(
                                            new Pose2d(-46, -56, Math.toRadians(0.00)),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(8, -58),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .turn(Math.toRadians(30))
                                    .build();

                            TrajectorySequence movement2Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement1Middle.end())
                                    .lineToSplineHeading(
                                            new Pose2d(37.72, -58, Math.toRadians(0.00)),
                                            SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(37.72, -34),
                                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(51, -34),
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
                                            new WaitUntilCommand(() -> robotSensed || time_since_start.seconds() > 22),
                                            new ParallelCommandGroup(
                                                    new LowOuttakeCommand(robot),
                                                    new DriveCommand(robot.driveSubsystem, movement2Middle)
                                            ),
                                            new WaitCommand(350),
                                            new InstantCommand(() -> robot.claw.releaseRight()),
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
            } else if (center) {
                if (parkRight) {
                    switch (recordedPropPosition) {
                        case LEFT:
                            TrajectorySequence movement1Left = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, -63.48, Math.toRadians(450.00)))
                                    .splineToConstantHeading(
                                            new Vector2d(-51, -41.70), Math.toRadians(90.00)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(-52.39, -50.41)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(-35.54, -50.41)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(-35.54, -10.5)
                                    )
                                    .lineToSplineHeading(
                                            new Pose2d(10, -10.5, Math.toRadians(-30))
                                    )
                                    .build();

                            TrajectorySequence movement2Left = robot.driveSubsystem.trajectorySequenceBuilder(movement1Left.end())
                                    .lineToSplineHeading(
                                            new Pose2d(37, -10.5, Math.toRadians(0.00))
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(37, -27.5)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(50, -27.5)
                                    )
                                    .build();

                            TrajectorySequence movement3Left = robot.driveSubsystem.trajectorySequenceBuilder(movement2Left.end())
                                    .lineToConstantHeading(
                                            new Vector2d(36.91, -27.5),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(36.52, -62),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(56.35, -62),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();

                            CommandScheduler.getInstance().schedule(
                                    new SequentialCommandGroup(
                                            new DriveCommand(robot.driveSubsystem, movement1Left),
                                            new WaitUntilCommand(() -> robotSensed || time_since_start.seconds() > 22),
                                            new ParallelCommandGroup(
                                                    new DriveCommand(robot.driveSubsystem, movement2Left),
                                                    new HighOuttakeCommand(robot)
                                            ),
                                            new WaitCommand(750),
                                            new InstantCommand(() -> robot.claw.releaseRight()),
                                            new WaitCommand(700),
                                            new ParallelCommandGroup(
                                                    new DriveCommand(robot.driveSubsystem, movement3Left),
                                                    new RestCommand(robot)
                                            )

                                    )
                            );
                            break;
                        case RIGHT:
                        case UNFOUND:
                            TrajectorySequence movement1Right = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, -63.48, Math.toRadians(450.00)))
                                    .splineTo(
                                            new Vector2d(-34.99, -37.93), Math.toRadians(45.00),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToSplineHeading(
                                            new Pose2d(-46.84, -43.28, Math.toRadians(90.00)),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(-46.55, -11),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToSplineHeading(
                                            new Pose2d(10, -11, Math.toRadians(-30)),
                                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();

                            TrajectorySequence movement2Right = robot.driveSubsystem.trajectorySequenceBuilder(movement1Right.end())
                                    .lineToSplineHeading(
                                            new Pose2d(36, -11, Math.toRadians(360.00)),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(36, -41),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(50, -41),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();

                            TrajectorySequence movement3Right = robot.driveSubsystem.trajectorySequenceBuilder(movement2Right.end())
                                    .lineToConstantHeading(
                                            new Vector2d(37.54, -35.0),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(37.54, -62),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(56.35, -62),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();

                            CommandScheduler.getInstance().schedule(
                                    new SequentialCommandGroup(
                                            new DriveCommand(robot.driveSubsystem, movement1Right),
                                            new WaitUntilCommand(() -> robotSensed || time_since_start.seconds() > 24),
                                            new ParallelCommandGroup(
                                                    new DriveCommand(robot.driveSubsystem, movement2Right),
                                                    new HighOuttakeCommand(robot)
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
                            TrajectorySequence movement1Middle = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, -63.48, Math.toRadians(450.00)))
                                    .splineToConstantHeading(
                                            new Vector2d(-44.50, -34.10), Math.toRadians(90.00),
                                            SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(-43.50, -45.46),
                                            SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(-52.39, -45.66),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(-53.78, -12.38),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToSplineHeading(
                                            new Pose2d(10, -10, Math.toRadians(-30.00)),
                                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();

                            TrajectorySequence movement2Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement1Middle.end())
                                    .lineToSplineHeading(
                                            new Pose2d(36, -11.46, Math.toRadians(0.00)),
                                            SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(36, -36.5),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(51.5, -36.5),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();

                            TrajectorySequence movement3Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement2Middle.end())
                                    .lineToConstantHeading(
                                            new Vector2d(38.1, -36.5),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(36.52, -62),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(56.35, -62),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();

                            CommandScheduler.getInstance().schedule(
                                    new SequentialCommandGroup(
                                            new DriveCommand(robot.driveSubsystem, movement1Middle),
                                            new WaitUntilCommand(() -> robotSensed || time_since_start.seconds() > 22),
                                            new ParallelCommandGroup(
                                                    new DriveCommand(robot.driveSubsystem, movement2Middle),
                                                    new HighOuttakeCommand(robot)
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
                } else if (parkLeft) {
                    switch (recordedPropPosition) {
                        case LEFT:
                            TrajectorySequence movement1Left = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, -63.48, Math.toRadians(450.00)))
                                    .splineToConstantHeading(
                                            new Vector2d(-51, -41.70), Math.toRadians(90.00)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(-52.39, -50.41)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(-35.54, -50.41)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(-35.54, -10.5)
                                    )
                                    .lineToSplineHeading(
                                            new Pose2d(10, -10.5, Math.toRadians(-30))
                                    )
                                    .build();

                            TrajectorySequence movement2Left = robot.driveSubsystem.trajectorySequenceBuilder(movement1Left.end())
                                    .lineToSplineHeading(
                                            new Pose2d(37, -10.5, Math.toRadians(0.00))
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(37, -27.5)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(50, -27.5)
                                    )
                                    .build();

                            TrajectorySequence movement3Left = robot.driveSubsystem.trajectorySequenceBuilder(movement2Left.end())
                                    .lineToConstantHeading(
                                            new Vector2d(36.91, -27.5),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(36.52, -10),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(56.35, -10),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();

                            CommandScheduler.getInstance().schedule(
                                    new SequentialCommandGroup(
                                            new DriveCommand(robot.driveSubsystem, movement1Left),
                                            new WaitUntilCommand(() -> robotSensed || time_since_start.seconds() > 22),
                                            new ParallelCommandGroup(
                                                    new DriveCommand(robot.driveSubsystem, movement2Left),
                                                    new HighOuttakeCommand(robot)
                                            ),
                                            new WaitCommand(750),
                                            new InstantCommand(() -> robot.claw.releaseRight()),
                                            new WaitCommand(700),
                                            new ParallelCommandGroup(
                                                    new DriveCommand(robot.driveSubsystem, movement3Left),
                                                    new RestCommand(robot)
                                            )

                                    )
                            );
                            break;
                        case RIGHT:
                        case UNFOUND:
                            TrajectorySequence movement1Right = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, -63.48, Math.toRadians(450.00)))
                                    .splineTo(
                                            new Vector2d(-34.99, -37.93), Math.toRadians(45.00),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToSplineHeading(
                                            new Pose2d(-46.84, -43.28, Math.toRadians(90.00)),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(-46.55, -11),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToSplineHeading(
                                            new Pose2d(10, -11, Math.toRadians(-30)),
                                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();

                            TrajectorySequence movement2Right = robot.driveSubsystem.trajectorySequenceBuilder(movement1Right.end())
                                    .lineToSplineHeading(
                                            new Pose2d(36, -11, Math.toRadians(360.00)),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(36, -41),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(50, -41),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();

                            TrajectorySequence movement3Right = robot.driveSubsystem.trajectorySequenceBuilder(movement2Right.end())
                                    .lineToConstantHeading(
                                            new Vector2d(37.54, -35.0),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(37.54, -12),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(56.35, -12),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();

                            CommandScheduler.getInstance().schedule(
                                    new SequentialCommandGroup(
                                            new DriveCommand(robot.driveSubsystem, movement1Right),
                                            new WaitUntilCommand(() -> robotSensed || time_since_start.seconds() > 24),
                                            new ParallelCommandGroup(
                                                    new DriveCommand(robot.driveSubsystem, movement2Right),
                                                    new HighOuttakeCommand(robot)
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
                            TrajectorySequence movement1Middle = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, -63.48, Math.toRadians(450.00)))
                                    .splineToConstantHeading(
                                            new Vector2d(-44.50, -34.10), Math.toRadians(90.00),
                                            SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(-43.50, -45.46),
                                            SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(-52.39, -45.66),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(-53.78, -12.38),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToSplineHeading(
                                            new Pose2d(10, -10, Math.toRadians(-30.00)),
                                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();

                            TrajectorySequence movement2Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement1Middle.end())
                                    .lineToSplineHeading(
                                            new Pose2d(36, -11.46, Math.toRadians(0.00)),
                                            SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(36, -36.5),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(51.5, -36.5),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();

                            TrajectorySequence movement3Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement2Middle.end())
                                    .lineToConstantHeading(
                                            new Vector2d(38.1, -36.5),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(36.52, -12),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(56.35, -12),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();

                            CommandScheduler.getInstance().schedule(
                                    new SequentialCommandGroup(
                                            new DriveCommand(robot.driveSubsystem, movement1Middle),
                                            new WaitUntilCommand(() -> robotSensed || time_since_start.seconds() > 22),
                                            new ParallelCommandGroup(
                                                    new DriveCommand(robot.driveSubsystem, movement2Middle),
                                                    new HighOuttakeCommand(robot)
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
                            TrajectorySequence movement1Left = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, -63.48, Math.toRadians(450.00)))
                                    .splineToConstantHeading(
                                            new Vector2d(-51, -41.70), Math.toRadians(90.00)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(-52.39, -50.41)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(-35.54, -50.41)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(-35.54, -10.5)
                                    )
                                    .lineToSplineHeading(
                                            new Pose2d(10, -10.5, Math.toRadians(-30))
                                    )
                                    .build();

                            TrajectorySequence movement2Left = robot.driveSubsystem.trajectorySequenceBuilder(movement1Left.end())
                                    .lineToSplineHeading(
                                            new Pose2d(37, -10.5, Math.toRadians(0.00))
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(37, -27.5)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(50, -27.5)
                                    )
                                    .build();

                            TrajectorySequence movement3Left = robot.driveSubsystem.trajectorySequenceBuilder(movement2Left.end())
                                    .back(6)
                                    .build();

                            CommandScheduler.getInstance().schedule(
                                    new SequentialCommandGroup(
                                            new DriveCommand(robot.driveSubsystem, movement1Left),
                                            new WaitUntilCommand(() -> robotSensed || time_since_start.seconds() > 22),
                                            new ParallelCommandGroup(
                                                    new DriveCommand(robot.driveSubsystem, movement2Left),
                                                    new HighOuttakeCommand(robot)
                                            ),
                                            new WaitCommand(750),
                                            new InstantCommand(() -> robot.claw.releaseRight()),
                                            new WaitCommand(700),
                                            new ParallelCommandGroup(
                                                    new DriveCommand(robot.driveSubsystem, movement3Left),
                                                    new RestCommand(robot)
                                            )

                                    )
                            );
                            break;
                        case RIGHT:
                        case UNFOUND:
                            TrajectorySequence movement1Right = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, -63.48, Math.toRadians(450.00)))
                                    .splineTo(
                                            new Vector2d(-34.99, -37.93), Math.toRadians(45.00),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToSplineHeading(
                                            new Pose2d(-46.84, -43.28, Math.toRadians(90.00)),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(-46.55, -11),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToSplineHeading(
                                            new Pose2d(10, -11, Math.toRadians(-30)),
                                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();

                            TrajectorySequence movement2Right = robot.driveSubsystem.trajectorySequenceBuilder(movement1Right.end())
                                    .lineToSplineHeading(
                                            new Pose2d(36, -11, Math.toRadians(360.00)),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(36, -41),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(50, -41),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();

                            TrajectorySequence movement3Right = robot.driveSubsystem.trajectorySequenceBuilder(movement2Right.end())
                                    .back(6)
                                    .build();

                            CommandScheduler.getInstance().schedule(
                                    new SequentialCommandGroup(
                                            new DriveCommand(robot.driveSubsystem, movement1Right),
                                            new WaitUntilCommand(() -> robotSensed || time_since_start.seconds() > 24),
                                            new ParallelCommandGroup(
                                                    new DriveCommand(robot.driveSubsystem, movement2Right),
                                                    new HighOuttakeCommand(robot)
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
                            TrajectorySequence movement1Middle = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(-40.11, -63.48, Math.toRadians(450.00)))
                                    .splineToConstantHeading(
                                            new Vector2d(-44.50, -34.10), Math.toRadians(90.00),
                                            SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(-43.50, -45.46),
                                            SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(-52.39, -45.66),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(-53.78, -12.38),
                                            SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToSplineHeading(
                                            new Pose2d(10, -10, Math.toRadians(-30.00)),
                                            SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();

                            TrajectorySequence movement2Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement1Middle.end())
                                    .lineToSplineHeading(
                                            new Pose2d(36, -11.46, Math.toRadians(0.00)),
                                            SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(36, -36.5),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .lineToConstantHeading(
                                            new Vector2d(51.5, -36.5),
                                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();

                            TrajectorySequence movement3Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement2Middle.end())
                                    .back(6)
                                    .build();

                            CommandScheduler.getInstance().schedule(
                                    new SequentialCommandGroup(
                                            new DriveCommand(robot.driveSubsystem, movement1Middle),
                                            new WaitUntilCommand(() -> robotSensed || time_since_start.seconds() > 22),
                                            new ParallelCommandGroup(
                                                    new DriveCommand(robot.driveSubsystem, movement2Middle),
                                                    new HighOuttakeCommand(robot)
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

        robotSensed = (robotProcessor.getSensedBoolean() != RedCenterRobotScan.Sensed.TRUE);
        robotSensedWall = (robotProcessor2.getSensedBoolean() != RedWallRobotScan.Sensed.TRUE);

        telemetry.addData("Robot Sensed: ", robotSensed);

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
