package org.firstinspires.ftc.teamcode.drive.newopmodes;

import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.common.commandbase.command.AutoDrop;
import org.firstinspires.ftc.teamcode.common.commandbase.command.Intake;
import org.firstinspires.ftc.teamcode.common.commandbase.command.OuttakePosition;
import org.firstinspires.ftc.teamcode.common.commandbase.command.TrajectorySequenceFollower;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.Drive;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

public class BlueLeft extends OpMode {
    private Robot robot;
    SampleMecanumDrive drive;
    private Drive drive2;
    private ElapsedTime time_since_start;
    TfodProcessor myTfodProcessor;
    int elementPosition;
    boolean USE_WEBCAM;
    private double loop;
    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
        drive2 = new Drive(new SampleMecanumDrive(hardwareMap), false);
        USE_WEBCAM = true;
        // Initialize TFOD before waitForStart.
        initTfod();
        // Wait for the match to begin.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        robot.a.armIntake();
        robot.claw.grab();
        robot.angle.rest();
    }

    @Override
    public void init_loop() {
        telemetryTfod();
    }

    public void start() {
        time_since_start = new ElapsedTime();
        if(elementPosition == 0) { //right

            TrajectorySequence dropPixelRight = drive.trajectorySequenceBuilder(new Pose2d(16.11, 63.82, Math.toRadians(90.00)))
                    .lineToSplineHeading(new Pose2d(11.93, 39.96, Math.toRadians(30.00)))
                    .build();

            TrajectorySequence backdropPixelRight = drive.trajectorySequenceBuilder(dropPixelRight.end())
                    .splineTo(new Vector2d(49.19, 28.82), Math.toRadians(0.00))
                    .build();

            TrajectorySequence preParkRight = drive.trajectorySequenceBuilder(backdropPixelRight.end())
                    .lineToConstantHeading(new Vector2d(49.54, 60.33))
                    .build();

            TrajectorySequence parkRight = drive.trajectorySequenceBuilder(preParkRight.end())
                    .lineToConstantHeading(new Vector2d(62.60, 60.33))
                    .build();

            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new WaitCommand(1000),
                            new Intake(robot),
                            new AutoDrop(robot),
                            new TrajectorySequenceFollower(drive2, dropPixelRight),
                            new WaitCommand(350),
                            new InstantCommand(() -> robot.claw.releaseLeft()),
                            new WaitCommand(350),
                            new ParallelCommandGroup(
                                    new InstantCommand(() -> robot.claw.grab()),
                                    new OuttakePosition(robot),
                                    new TrajectorySequenceFollower(drive2, backdropPixelRight)
                            ),
                            new WaitCommand(350),
                            new InstantCommand(() -> robot.claw.releaseRight()),
                            new WaitCommand(350),
                            new ParallelCommandGroup(
                                    new InstantCommand(() -> robot.claw.grab()),
                                    new InstantCommand(() -> robot.a.armCoast())
                            ),
                            new TrajectorySequenceFollower(drive2, preParkRight),
                            new TrajectorySequenceFollower(drive2, parkRight)
                    )
            );
        } else if(elementPosition == 1) { //middle
            TrajectorySequence dropPixelMiddle = drive.trajectorySequenceBuilder(new Pose2d(16.11, 63.82, Math.toRadians(90.00)))
                    .lineToConstantHeading(new Vector2d(12.45, 36.48))
                    .build();

            TrajectorySequence backdropPixelMiddle = drive.trajectorySequenceBuilder(dropPixelMiddle.end())
                    .splineTo(new Vector2d(49.19, 35.78), Math.toRadians(0.00))
                    .build();

            TrajectorySequence preParkMiddle = drive.trajectorySequenceBuilder(backdropPixelMiddle.end())
                    .lineToConstantHeading(new Vector2d(48.84, 59.64))
                    .build();

            TrajectorySequence parkMiddle = drive.trajectorySequenceBuilder(preParkMiddle.end())
                    .lineToConstantHeading(new Vector2d(62.25, 59.64))
                    .build();

            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new WaitCommand(1000),
                            new Intake(robot),
                            new AutoDrop(robot),
                            new TrajectorySequenceFollower(drive2, dropPixelMiddle),
                            new WaitCommand(350),
                            new InstantCommand(() -> robot.claw.releaseLeft()),
                            new WaitCommand(350),
                            new ParallelCommandGroup(
                                    new InstantCommand(() -> robot.claw.grab()),
                                    new OuttakePosition(robot),
                                    new TrajectorySequenceFollower(drive2, backdropPixelMiddle)
                            ),
                            new WaitCommand(350),
                            new InstantCommand(() -> robot.claw.releaseRight()),
                            new WaitCommand(350),
                            new ParallelCommandGroup(
                                    new InstantCommand(() -> robot.claw.grab()),
                                    new InstantCommand(() -> robot.a.armCoast())
                            ),
                            new TrajectorySequenceFollower(drive2, preParkMiddle),
                            new TrajectorySequenceFollower(drive2, parkMiddle)
                    )
            );

        } else if(elementPosition == 1) { //left
            TrajectorySequence dropPixelLeft = drive.trajectorySequenceBuilder(new Pose2d(16.11, 63.82, Math.toRadians(90.00)))
                    .lineToConstantHeading(new Vector2d(22.20, 42.57))
                    .build();

            TrajectorySequence backdropPixelLeft = drive.trajectorySequenceBuilder(dropPixelLeft.end())
                    .splineTo(new Vector2d(48.84, 41.53), Math.toRadians(0.00))
                    .build();

            TrajectorySequence preParkLeft = drive.trajectorySequenceBuilder(backdropPixelLeft.end())
                    .lineToConstantHeading(new Vector2d(48.67, 60.33))
                    .build();

            TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(preParkLeft.end())
                    .lineToConstantHeading(new Vector2d(62.95, 60.16))
                    .build();

            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new WaitCommand(1000),
                            new Intake(robot),
                            new AutoDrop(robot),
                            new TrajectorySequenceFollower(drive2, dropPixelLeft),
                            new WaitCommand(350),
                            new InstantCommand(() -> robot.claw.releaseLeft()),
                            new WaitCommand(350),
                            new ParallelCommandGroup(
                                    new InstantCommand(() -> robot.claw.grab()),
                                    new OuttakePosition(robot),
                                    new TrajectorySequenceFollower(drive2, backdropPixelLeft)
                            ),
                            new WaitCommand(350),
                            new InstantCommand(() -> robot.claw.releaseRight()),
                            new WaitCommand(350),
                            new ParallelCommandGroup(
                                    new InstantCommand(() -> robot.claw.grab()),
                                    new InstantCommand(() -> robot.a.armCoast())
                            ),
                            new TrajectorySequenceFollower(drive2, preParkLeft),
                            new TrajectorySequenceFollower(drive2, parkLeft)
                    )
            );
        }
    }
    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        robot.a.loop();
        drive2.update();

        double time = System.currentTimeMillis();
        telemetry.addData("Loop: ", time - loop);
        telemetry.addData("Arm Position: ", robot.a.getCachePos());
        loop = time;

        telemetry.update();
    }

    private void initTfod() {
        TfodProcessor.Builder myTfodProcessorBuilder;
        VisionPortal.Builder myVisionPortalBuilder;
        VisionPortal myVisionPortal;

        myTfodProcessorBuilder = new TfodProcessor.Builder();
        myTfodProcessorBuilder.setModelFileName("BLUETURKEY.tflite");
        myTfodProcessorBuilder.setModelLabels(JavaUtil.createListWith("TSE"));
        myTfodProcessorBuilder.setModelAspectRatio(16 / 9);
        myTfodProcessor = myTfodProcessorBuilder.build();
        myVisionPortalBuilder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam"));
        } else {
            myVisionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
        }
        myVisionPortalBuilder.addProcessor(myTfodProcessor);
        myVisionPortal = myVisionPortalBuilder.build();
    }
    private void telemetryTfod() {
        boolean sensedElement;
        List<Recognition> myTfodRecognitions;
        Recognition myTfodRecognition;
        float x;
        float y;

        sensedElement = false;
        myTfodRecognitions = (List<Recognition>) myTfodProcessor.getRecognitions();
        telemetry.addData("# Objects Detected", JavaUtil.listLength(myTfodRecognitions));
        for (Recognition myTfodRecognition_item : myTfodRecognitions) {
            myTfodRecognition = myTfodRecognition_item;
            if (myTfodRecognition.getConfidence() * 100 > 70) {
                telemetry.addLine("");
                telemetry.addData("Image", myTfodRecognition.getLabel() + " (" + JavaUtil.formatNumber(myTfodRecognition.getConfidence() * 100, 0) + " % Conf.)");
                x = (myTfodRecognition.getLeft() + myTfodRecognition.getRight()) / 2;
                y = (myTfodRecognition.getTop() + myTfodRecognition.getBottom()) / 2;
                telemetry.addData("- Position", JavaUtil.formatNumber(x, 0) + ", " + JavaUtil.formatNumber(y, 0));
                telemetry.addData("- Size", JavaUtil.formatNumber(myTfodRecognition.getWidth(), 0) + " x " + JavaUtil.formatNumber(myTfodRecognition.getHeight(), 0));
                if (x > 500) {
                    sensedElement = true;
                    elementPosition = 0;
                    telemetry.addData("Team Element Detection: ", "Right");
                } else if (x >= 190 && x <= 250) {
                    sensedElement = true;
                    elementPosition = 1;
                    telemetry.addData("Team Element Detection: ", "Middle");
                }
            }
        }
        if (!sensedElement) {
            elementPosition = 2;
            telemetry.addData("Team Element Detection: ", "Left");
        }
    }
    //0: right
    //1: middle
    //2: left
}
