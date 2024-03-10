package org.firstinspires.ftc.teamcode.Drive.OpModes.States.Blue;

import static java.lang.Thread.sleep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.Drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.TrajectorySequences.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.BlueStackCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.DriveCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.OuttakeCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.RestCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.SlidesDownCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.SuperHighOuttakeCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.TwoPixelDropCommand;
import org.firstinspires.ftc.teamcode.Utility.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility.Vision.Prop.NewBlueLeftProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

@Autonomous
public class SixtyBlueLeft extends OpMode {
    private VisionPortal visionPortal;
    private NewBlueLeftProcessor colorMassDetectionProcessor;

    private AprilTagProcessor aprilTag;
    private AprilTagDetection tag;
    private VisionPortal visionPortal2;
    public Vector2d cameraOffset = new Vector2d(
            -3,
            7.5);


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

        colorMassDetectionProcessor = new NewBlueLeftProcessor();
        colorMassDetectionProcessor.setDetectionColor(false); //false is blue, true is red

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
        visionPortal.close();
        initAprilTag();
        time_since_start = new ElapsedTime();
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.stopLiveView();
            visionPortal.stopStreaming();
        }

        NewBlueLeftProcessor.PropPositions recordedPropPosition = colorMassDetectionProcessor.getPropLocation();
        robot.driveSubsystem.setPoseEstimate(new Pose2d(16.14, 63.32, Math.toRadians(-90.00)));
        switch (recordedPropPosition) {
            case LEFT:
                TrajectorySequence movement1Left = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(16.14, 63.32, Math.toRadians(-90.00)))
                        .splineToConstantHeading(
                                new Vector2d(27.8, 43.31), Math.toRadians(270.00),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToConstantHeading(
                                new Vector2d(27.52, 56.52),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence movement2Left = robot.driveSubsystem.trajectorySequenceBuilder(movement1Left.end())
                        .splineToSplineHeading(
                                new Pose2d(52.3, 41, Math.toRadians(0.00)), Math.toRadians(0.00),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence movement3Left = robot.driveSubsystem.trajectorySequenceBuilder(movement2Left.end())
                        .lineToConstantHeading(
                                new Vector2d(46, 40.5),
                                SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToConstantHeading(
                                new Vector2d(25, 13),
                                SampleMecanumDrive.getVelocityConstraint(27, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence movement4Left = robot.driveSubsystem.trajectorySequenceBuilder(movement3Left.end())
                        .lineToLinearHeading(
                                new Pose2d(-58.2, 13.45, Math.toRadians(0)),
                                SampleMecanumDrive.getVelocityConstraint(29, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )

                        .lineToLinearHeading(
                                new Pose2d(-58.5, 13.5, Math.toRadians(-0.03)),
                                SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )


                        .build();
                TrajectorySequence movement5Left = robot.driveSubsystem.trajectorySequenceBuilder(movement4Left.end())
                        .lineToConstantHeading(
                                new Vector2d(25, 13.5),
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();
                TrajectorySequence movement6Left = robot.driveSubsystem.trajectorySequenceBuilder(movement5Left.end())
                        .lineToConstantHeading(
                                new Vector2d(47.9, 28.8),
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();
                TrajectorySequence movement7Left = robot.driveSubsystem.trajectorySequenceBuilder(movement6Left.end())
                        .lineToConstantHeading(
                                new Vector2d(46, 27),
                                SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToConstantHeading(
                                new Vector2d(42.3, 26),
                                SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
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
                                new WaitCommand(400),
                                new InstantCommand(() -> robot.claw.releaseLeft()),
                                new WaitCommand(400),
                                new ParallelCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement3Left),
                                        new WaitCommand(200),
                                        new BlueStackCommand(robot)
                                ),
                                new InstantCommand(() -> robot.claw.customLeft(1)),
                                new DriveCommand(robot.driveSubsystem, movement4Left),
                                new WaitCommand(300),
                                new InstantCommand(() -> robot.claw.grabBoth()),
                                new DriveCommand(robot.driveSubsystem, movement5Left),
                                new ParallelCommandGroup(

                                        new DriveCommand(robot.driveSubsystem, movement6Left),
                                        new SuperHighOuttakeCommand(robot)
                                ),

                                new TwoPixelDropCommand(robot),
                                new WaitCommand(1000),
                                new InstantCommand(() -> robot.claw.customLeft(0.7)),
                                new WaitCommand(300),
                                new DriveCommand(robot.driveSubsystem, movement7Left),
                                new SlidesDownCommand(robot)



                        )
                );
                break;
            case RIGHT:
            case UNFOUND:
                TrajectorySequence movement1Right = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(16.14, 63.32, Math.toRadians(-90.00)))
                        .splineTo(
                                new Vector2d(9.8, 35.78), Math.toRadians(220.00),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToSplineHeading(
                                new Pose2d(19.31, 46.84, Math.toRadians(270.00)),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence movement2Right = robot.driveSubsystem.trajectorySequenceBuilder(movement1Right.end())
                        .splineToSplineHeading(
                                new Pose2d(51.8, 28.4, Math.toRadians(0.00)), Math.toRadians(0.00),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence movement3Right = robot.driveSubsystem.trajectorySequenceBuilder(movement2Right.end())
                        .lineToConstantHeading(
                                new Vector2d(46, 28),
                                SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToConstantHeading(
                                new Vector2d(25, 13),
                                SampleMecanumDrive.getVelocityConstraint(27, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence movement4Right = robot.driveSubsystem.trajectorySequenceBuilder(movement3Right.end())
                        .lineToLinearHeading(
                                new Pose2d(-58.2, 13.45, Math.toRadians(0)),
                                SampleMecanumDrive.getVelocityConstraint(29, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )

                        .lineToLinearHeading(
                                new Pose2d(-58.5, 13.5, Math.toRadians(-0.03)),
                                SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )


                        .build();
                TrajectorySequence movement5Right = robot.driveSubsystem.trajectorySequenceBuilder(movement4Right.end())
                        .lineToConstantHeading(
                                new Vector2d(25, 13.5),
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();
                TrajectorySequence movement6Right = robot.driveSubsystem.trajectorySequenceBuilder(movement5Right.end())
                        .lineToConstantHeading(
                                new Vector2d(47.9, 28.8),
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();
                TrajectorySequence movement7Right = robot.driveSubsystem.trajectorySequenceBuilder(movement6Right.end())
                        .lineToConstantHeading(
                                new Vector2d(46, 27),
                                SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToConstantHeading(
                                new Vector2d(42.3, 26),
                                SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
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
                                new WaitCommand(400),
                                new InstantCommand(() -> robot.claw.releaseLeft()),
                                new WaitCommand(400),
                                new ParallelCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement3Right),
                                        new WaitCommand(200),
                                        new BlueStackCommand(robot)
                                ),
                                new InstantCommand(() -> robot.claw.customLeft(1)),
                                new DriveCommand(robot.driveSubsystem, movement4Right),
                                new WaitCommand(300),
                                new InstantCommand(() -> robot.claw.grabBoth()),
                                new DriveCommand(robot.driveSubsystem, movement5Right),
                                new ParallelCommandGroup(

                                        new DriveCommand(robot.driveSubsystem, movement6Right),
                                        new SuperHighOuttakeCommand(robot)
                                ),

                                new TwoPixelDropCommand(robot),
                                new WaitCommand(1000),
                                new InstantCommand(() -> robot.claw.customLeft(0.7)),
                                new WaitCommand(300),
                                new DriveCommand(robot.driveSubsystem, movement7Right),
                                new SlidesDownCommand(robot)



                        )
                );
                break;
            case MIDDLE:
                TrajectorySequence movement1Middle = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(16.14, 63.32, Math.toRadians(-90.00)))
                        .splineToConstantHeading(
                                new Vector2d(20.25, 35.2), Math.toRadians(270.00),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .lineToConstantHeading(
                                new Vector2d(20.25, 50),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence movement2Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement1Middle.end())
                        .splineToSplineHeading(
                                new Pose2d(52.5, 33, Math.toRadians(0.00)), Math.toRadians(0.00),
                                SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence movement3Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement2Middle.end())
                        .lineToConstantHeading(
                                new Vector2d(46, 33.2),
                                SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence extraMovement = robot.driveSubsystem.trajectorySequenceBuilder(movement3Middle.end())
                        .lineToConstantHeading(
                                new Vector2d(25, 13),
                                SampleMecanumDrive.getVelocityConstraint(27, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence movement4Middle = robot.driveSubsystem.trajectorySequenceBuilder(extraMovement.end())
                        .lineToLinearHeading(
                                new Pose2d(-56, 13.3, Math.toRadians(0)),
                                SampleMecanumDrive.getVelocityConstraint(29, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )

                        .lineToLinearHeading(
                                new Pose2d(-59.47, 13.3, Math.toRadians(0)),
                                SampleMecanumDrive.getVelocityConstraint(7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )


                        .build();
                TrajectorySequence movement5Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement4Middle.end())
                        .lineToConstantHeading(
                                new Vector2d(25, 13.5),
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence movement6Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement5Middle.end())
                        .lineToConstantHeading(
                                new Vector2d(47.9, 31),
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                TrajectorySequence movement7Middle = robot.driveSubsystem.trajectorySequenceBuilder(movement6Middle.end())
                        .lineToConstantHeading(
                                new Vector2d(47, 31),
                                SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .back(10)
                        .strafeRight(18)
                        .forward(10)
                        .build();


                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new DriveCommand(robot.driveSubsystem, movement1Middle),
                                new ParallelCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement2Middle),
                                        new OuttakeCommand(robot)
                                ),
                                new WaitCommand(400),
                                new InstantCommand(() -> robot.claw.releaseLeft()),
                                new WaitCommand(400),
                                new DriveCommand(robot.driveSubsystem, movement3Middle),
                                new ParallelCommandGroup(
                                        new BlueStackCommand(robot),
                                        new DriveCommand(robot.driveSubsystem, extraMovement)
                                ),
                                new WaitCommand(200),
                                new InstantCommand(() -> robot.claw.customLeft(1)),
                                new WaitCommand(500),
                                new DriveCommand(robot.driveSubsystem, movement4Middle),
                                new WaitCommand(300),
                                new InstantCommand(() -> robot.claw.grabBoth()),
                                new DriveCommand(robot.driveSubsystem, movement5Middle),
                                new ParallelCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement6Middle),
                                        new SuperHighOuttakeCommand(robot)
                                ),
                                new WaitCommand(400),
                                new TwoPixelDropCommand(robot),
                                new WaitCommand(400),
                              new InstantCommand(() -> robot.angleOfArm.customAngle(0.5)),
                                new WaitCommand(350),
                                new InstantCommand(() -> robot.claw.customLeft(0.85)),
                                new WaitCommand(350),
                                new InstantCommand(() -> robot.claw.customLeft(0.9)),
                                new ParallelCommandGroup(
                                        new DriveCommand(robot.driveSubsystem, movement7Middle),
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

        if(aprilTag.getDetections().size() > 0) {
            tag = aprilTag.getDetections().get(0);
        }
        if(!aprilTag.getDetections().isEmpty()) {
            telemetry.addData("Yipee: ", "Relocalized.");
            robot.driveSubsystem.setPoseEstimate(telemetryAprilTag(tag, robot.driveSubsystem.getPoseEstimate().getHeading()));
        }

        double time = System.currentTimeMillis();
        telemetry.addData("Time Elapsed: ", time_since_start);
        telemetry.addData("Current Loop Time: ", time - loop);

        loop = time;
        telemetry.update();
    }

    @Override
    public void stop() {
        visionPortal2.close();
        telemetry.addLine("Closed Camera.");
        telemetry.update();
        CommandScheduler.getInstance().reset();
    }
    private void initAprilTag() {

        aprilTag = new AprilTagProcessor.Builder()
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506) //TODO: tune this
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam"));
        builder.addProcessor(aprilTag);
        builder.build();
    }

    private Pose2d telemetryAprilTag(AprilTagDetection detection, double robotHeading) {
        double x = -detection.ftcPose.x - cameraOffset.getX();
        double y = -detection.ftcPose.y - cameraOffset.getY();

        double botHeading = -robot.driveSubsystem.getPoseEstimate().getHeading();


        double x2 = x * Math.cos(botHeading) + y * Math.sin(botHeading);
        double y2 = x * -Math.sin(botHeading) + y * Math.cos(botHeading);
        double absX;
        double absY;

        VectorF tagPosition = getCenterStageTagLibrary().lookupTag(detection.id).fieldPosition;
        if (detection.metadata.id <= 6) {
            absX = tagPosition.get(0) + y2;
            absY = tagPosition.get(1) - x2;

        } else {
            absX = tagPosition.get(0) - y2;
            absY = tagPosition.get(1) + x2;

        }
        return new Pose2d(absX, absY, botHeading);
        //telemetry.addData("Position: ", fieldCentricPosition);
    }



    public static AprilTagLibrary getCenterStageTagLibrary()
    {
        return new AprilTagLibrary.Builder()
                .addTag(1, "BlueAllianceLeft",
                        2, new VectorF(61.75f, 41.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(2, "BlueAllianceCenter",
                        2, new VectorF(61.75f, 35.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(3, "BlueAllianceRight",
                        2, new VectorF(61.75f, 29.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(4, "RedAllianceLeft",
                        2, new VectorF(61.75f, -29.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(5, "RedAllianceCenter",
                        2, new VectorF(61.75f, -35.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(6, "RedAllianceRight",
                        2, new VectorF(61.75f, -41.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(7, "RedAudienceWallLarge",
                        5, new VectorF(-70.25f, -40.625f, 5.5f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .addTag(8, "RedAudienceWallSmall",
                        2, new VectorF(-70.25f, -35.125f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .addTag(9, "BlueAudienceWallSmall",
                        2, new VectorF(-70.25f, 35.125f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .addTag(10, "BlueAudienceWallLarge",
                        5, new VectorF(-70.25f, 40.625f, 5.5f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .build();
    }
}