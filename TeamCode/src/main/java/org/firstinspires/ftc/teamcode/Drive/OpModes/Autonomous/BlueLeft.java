package org.firstinspires.ftc.teamcode.Drive.OpModes.Autonomous;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.TrajectorySequences.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.DriveCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.OuttakeCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.RestCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.TapeDropCommand;
import org.firstinspires.ftc.teamcode.Utility.Hardware.RobotHardware;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous
public class BlueLeft extends OpMode {
    private RobotHardware robot;
    private ElapsedTime time_since_start;
    private double loop;

    TfodProcessor myTfodProcessor;
    int PeacockPosition;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        robot = new RobotHardware(hardwareMap);

        CommandScheduler.getInstance().registerSubsystem(robot.armSystem);
        CommandScheduler.getInstance().registerSubsystem(robot.claw);
        CommandScheduler.getInstance().registerSubsystem(robot.angleOfArm);
        CommandScheduler.getInstance().registerSubsystem(robot.driveSubsystem);
        CommandScheduler.getInstance().registerSubsystem(robot.slidesSubsystem);

        initTfod();
        telemetry.addLine("Getting ready to run BlueLeft autonomous. Camera not initialized.");
        telemetry.update();

        robot.claw.grabBoth();

    }
    @Override
    public void init_loop() {
        telemetry.addData("Successful: ", "Ready for BlueLeft (Backdrop Side)");
        telemetry.addData("Ready to Run: ", "2 pixel autonomous. All subsystems initialized.");
        telemetryTfod();
        CommandScheduler.getInstance().run();
        robot.armSystem.loop();
        telemetry.update();
    }

    @Override
    public void start() {
        time_since_start = new ElapsedTime();
        robot.driveSubsystem.setPoseEstimate(new Pose2d(15.76, 63.99, Math.toRadians(-90.00)));

        if (PeacockPosition == 2) { //CASE: LEFT
            TrajectorySequence backdropLeft = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(15.76, 63.99, Math.toRadians(-90.00)))
                    .splineTo(
                            new Vector2d(49.19, 40.14), Math.toRadians(0.00),
                            SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .build();

            TrajectorySequence tapeLeft = robot.driveSubsystem.trajectorySequenceBuilder(backdropLeft.end())
                    .lineToSplineHeading(
                            new Pose2d(33.87, 36.48, Math.toRadians(0.00)),
                            SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .build();

            TrajectorySequence parkLeft = robot.driveSubsystem.trajectorySequenceBuilder(tapeLeft.end())
                    .lineToConstantHeading(new Vector2d(39.61, 36.83))
                    .lineToConstantHeading(new Vector2d(38.57, 63.29))
                    .lineToConstantHeading(
                            new Vector2d(58.94, 63.12),
                            SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .build();

            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new ParallelCommandGroup(
                                    new DriveCommand(robot.driveSubsystem, backdropLeft),
                                    //                               new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequenceNotAsync(backdropLeft)),
                                    new OuttakeCommand(robot)
                            ),
                            new WaitCommand(3500),
                            new InstantCommand(() -> robot.claw.autoReleaseLeft()),
                            new WaitCommand(350),
                            new ParallelCommandGroup(
                                    new DriveCommand(robot.driveSubsystem, tapeLeft),
                                    //new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequenceNotAsync(tapeLeft)),
                                    new TapeDropCommand(robot)
                            ),
                            new WaitCommand(350),
                            new InstantCommand(() -> robot.claw.releaseRight()),
                            new WaitCommand(1000),
                            new RestCommand(robot),
                            new WaitCommand(350),
                            new DriveCommand(robot.driveSubsystem, parkLeft)
                            //new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequenceNotAsync(parkLeft))

                    )
            );
        } else if (PeacockPosition == 1) { //CASE: MIDDLE
            TrajectorySequence backdropMiddle = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(15.76, 63.99, Math.toRadians(-90.00)))
                    .splineTo(
                            new Vector2d(49.54, 35.61), Math.toRadians(0.00),
                            SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .build();

            TrajectorySequence tapeMiddle = robot.driveSubsystem.trajectorySequenceBuilder(backdropMiddle.end())
                    .lineToSplineHeading(
                            new Pose2d(27.25, 26.55, Math.toRadians(0.00)),
                            SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .build();

            TrajectorySequence parkMiddle = robot.driveSubsystem.trajectorySequenceBuilder(tapeMiddle.end())
                    .lineToConstantHeading(new Vector2d(36.48, 26.21))
                    .lineToConstantHeading(new Vector2d(35.78, 63.29))
                    .lineToConstantHeading(
                            new Vector2d(58.94, 63.12),
                            SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .build();

            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new ParallelCommandGroup(
                                    new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequenceNotAsync(backdropMiddle)),
                                    new OuttakeCommand(robot)
                            ),
                            new WaitCommand(3500),
                            new InstantCommand(() -> robot.claw.autoReleaseLeft()),
                            new WaitCommand(350),
                            new ParallelCommandGroup(
                                    new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequenceNotAsync(tapeMiddle)),
                                    new TapeDropCommand(robot)
                            ),
                            new WaitCommand(350),
                            new InstantCommand(() -> robot.claw.releaseRight()),
                            new WaitCommand(1000),
                            new RestCommand(robot),
                            new WaitCommand(350),
                            new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequenceNotAsync(parkMiddle))

                    )
            );
        } else if (PeacockPosition == 3) { //CASE: RIGHT
            TrajectorySequence backdropRight = robot.driveSubsystem.trajectorySequenceBuilder(new Pose2d(15.76, 63.99, Math.toRadians(-90.00)))
                    .splineTo(
                            new Vector2d(49.19, 29.69), Math.toRadians(0.00),
                            SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .build();

            TrajectorySequence tapeRight = robot.driveSubsystem.trajectorySequenceBuilder(backdropRight.end())
                    .lineToSplineHeading(
                            new Pose2d(11.41, 38.22, Math.toRadians(0.00)),
                            SampleMecanumDrive.getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .build();

            TrajectorySequence parkRight = robot.driveSubsystem.trajectorySequenceBuilder(tapeRight.end())
                    .lineToConstantHeading(new Vector2d(20.81, 37.7))
                    .lineToConstantHeading(new Vector2d(35.78, 63.29))
                    .lineToConstantHeading(
                            new Vector2d(58.94, 63.12),
                            SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .build();

            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new ParallelCommandGroup(
                                    new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequenceNotAsync(backdropRight)),
                                    new OuttakeCommand(robot)
                            ),
                            new WaitCommand(3500),
                            new InstantCommand(() -> robot.claw.autoReleaseLeft()),
                            new WaitCommand(350),
                            new ParallelCommandGroup(
                                    new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequenceNotAsync(tapeRight)),
                                    new TapeDropCommand(robot)
                            ),
                            new WaitCommand(350),
                            new InstantCommand(() -> robot.claw.releaseRight()),
                            new WaitCommand(1000),
                            new RestCommand(robot),
                            new WaitCommand(350),
                            new InstantCommand(() -> robot.driveSubsystem.followTrajectorySequenceNotAsync(parkRight))

                    )
            );
        }

    }
    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        robot.armSystem.loop();
        robot.driveSubsystem.update();
        robot.slidesSubsystem.loop();

        double time = System.currentTimeMillis();
        telemetry.addData("Time Elapsed: ", time_since_start);
        telemetry.addData("Current Loop Time: ", time - loop);

        robot.currentUpdate(telemetry);
        robot.pidArmUpdateTelemetry(telemetry);

        loop = time;

        telemetry.update();
    }

    @Override
    public void stop() {
        CommandScheduler.getInstance().reset();
    }

    private void initTfod() {
        TfodProcessor.Builder myTfodProcessorBuilder;
        VisionPortal.Builder myVisionPortalBuilder;
        VisionPortal myVisionPortal;

        myTfodProcessorBuilder = new TfodProcessor.Builder();
        myTfodProcessorBuilder.setModelFileName("NewBlueTurkey.tflite");
        myTfodProcessorBuilder.setModelLabels(JavaUtil.createListWith("TSE"));
        myTfodProcessorBuilder.setModelAspectRatio(16 / 9);
        myTfodProcessor = myTfodProcessorBuilder.build();
        myVisionPortalBuilder = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .setCameraResolution(new Size(1280, 720))
                .addProcessor(myTfodProcessor)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true);
        myVisionPortal = myVisionPortalBuilder.build();
    }

    private void telemetryTfod() {
        boolean sensedPeacockBoolean;
        List < Recognition > myTfodRecognitions;
        Recognition myTfodRecognition;
        float x;
        float y;

        sensedPeacockBoolean = false;
        myTfodRecognitions = myTfodProcessor.getRecognitions();
        //telemetry.addData("Objects Detected", JavaUtil.listLength(myTfodRecognitions));
        for (Recognition myTfodRecognition_item: myTfodRecognitions) {
            myTfodRecognition = myTfodRecognition_item;
            if (myTfodRecognition.getConfidence() * 100 > 77) {
                //telemetry.addLine("");
                telemetry.addData("Image: ", myTfodRecognition.getLabel() + " (" + JavaUtil.formatNumber(myTfodRecognition.getConfidence() * 100, 0) + " % Conf.)");
                x = (myTfodRecognition.getLeft() + myTfodRecognition.getRight()) / 2;
                y = (myTfodRecognition.getTop() + myTfodRecognition.getBottom()) / 2;
                //telemetry.addData("- Position", JavaUtil.formatNumber(x, 0) + ", " + JavaUtil.formatNumber(y, 0));
                //telemetry.addData("- Size", JavaUtil.formatNumber(myTfodRecognition.getWidth(), 0) + " x " + JavaUtil.formatNumber(myTfodRecognition.getHeight(), 0));
                if (x > 500) {
                    sensedPeacockBoolean = true;
                    PeacockPosition = 1;
                    telemetry.addData("PeacockPosition: ", "Middle");
                } else if (x >= 190 && x <= 250) {
                    sensedPeacockBoolean = true;
                    PeacockPosition = 2;
                    telemetry.addData("PeacockPosition: ", "Left");
                }
            }
        }
        if (!sensedPeacockBoolean) {
            PeacockPosition = 0;
            telemetry.addData("PeacockPosition: ", "Right");
        }
    }
}