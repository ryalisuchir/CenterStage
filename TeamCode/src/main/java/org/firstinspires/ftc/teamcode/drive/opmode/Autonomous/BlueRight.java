//package org.firstinspires.ftc.teamcode.drive.opmode.Autonomous;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.arcrobotics.ftclib.command.CommandScheduler;
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.ParallelCommandGroup;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.sun.tools.javac.util.List;
//
//import org.firstinspires.ftc.robotcore.external.JavaUtil;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
//import org.firstinspires.ftc.teamcode.common.commandbase.command.TapeDrop;
//import org.firstinspires.ftc.teamcode.common.commandbase.command.Intake;
//import org.firstinspires.ftc.teamcode.common.commandbase.command.OuttakePosition;
//import org.firstinspires.ftc.teamcode.common.commandbase.command.TrajectorySequenceFollower;
//import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.Drive;
//import org.firstinspires.ftc.teamcode.common.hardware.Robot;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.tfod.TfodProcessor;
//
//public class BlueRight extends OpMode {
//    private Robot robot;
//    private Drive drive;
//    private ElapsedTime time_since_start;
//    TfodProcessor myTfodProcessor;
//    int elementPosition;
//    boolean USE_WEBCAM;
//    private double loop;
//    @Override
//    public void init() {
//        robot = new Robot(hardwareMap);
//        drive = new Drive(new SampleMecanumDrive(hardwareMap), false);
//        USE_WEBCAM = true;
//        initTfod();
//        telemetry.addData("D.S. Preview: ", "Ready for BlueRight (NOT Backdrop Side)");
//        telemetry.update();
//
//        robot.a.armIntake();
//        robot.claw.grabBoth();
//        robot.angle.rest();
//    }
//
//    @Override
//    public void init_loop() {
//        telemetryTfod();
//    }
//
//    public void start() {
//        time_since_start = new ElapsedTime();
//        if(elementPosition == 0) { //right
//            TrajectorySequence firstGeneralRight = drive.trajectorySequenceBuilder(new Pose2d(-32.13, 63.64, Math.toRadians(90.00)))
//                    .lineToConstantHeading(new Vector2d(-38.39, 63.82))
//                    .build();
//
//            TrajectorySequence dropPixelRight = drive.trajectorySequenceBuilder(firstGeneralRight.end())
//                    .lineToSplineHeading(new Pose2d(-47.27, 41.88, Math.toRadians(90.00)))
//                    .build();
//
//            TrajectorySequence backdropPixelRight = drive.trajectorySequenceBuilder(dropPixelRight.end())
//                    .lineToConstantHeading(new Vector2d(-32.65, 41.53))
//                    .lineToConstantHeading(new Vector2d(-32.13, 12.10))
//                    .build();
//
//            TrajectorySequence preParkRight = drive.trajectorySequenceBuilder(backdropPixelRight.end())
//                    .lineToSplineHeading(new Pose2d(-57.20, 11.75, Math.toRadians(0.00)))
//                    .build();
//
//            TrajectorySequence parkRight = drive.trajectorySequenceBuilder(preParkRight.end())
//                    .lineToConstantHeading(new Vector2d(62.42, 11.58))
//                    .build();
//
//            CommandScheduler.getInstance().schedule(
//                    new SequentialCommandGroup(
//                            new WaitCommand(1000),
//                            new ParallelCommandGroup(
//                                    new TrajectorySequenceFollower(drive, firstGeneralRight),
//                                    new TapeDrop(robot)
//                            ),
//                            new TrajectorySequenceFollower(drive, dropPixelRight),
//                            new WaitCommand(350),
//                            new InstantCommand(() -> robot.claw.releaseLeft()),
//                            new WaitCommand(350),
//                            new ParallelCommandGroup(
//                                    new InstantCommand(() -> robot.a.armCoast()),
//                                    new InstantCommand(() -> robot.angle.intake()),
//                                    new TrajectorySequenceFollower(drive, backdropPixelRight)
//                            ),
//                            new WaitCommand(350),
//                            new InstantCommand(() -> robot.claw.grabLeft()),
//                            new WaitCommand(350),
//                            new InstantCommand(() -> robot.angle.outtake()),
//                            new TrajectorySequenceFollower(drive, preParkRight),
//                            new TrajectorySequenceFollower(drive, parkRight)
//                    )
//            );
//        } else if(elementPosition == 1) { //middle
//            TrajectorySequence firstGeneralMiddle = drive.trajectorySequenceBuilder(new Pose2d(-32.13, -64.86, Math.toRadians(-90.00)))
//                    .lineToConstantHeading(new Vector2d(-38.22, -64.16))
//                    .build();
//
//            TrajectorySequence dropPixelMiddle = drive.trajectorySequenceBuilder(firstGeneralMiddle.end())
//                    .lineToConstantHeading(new Vector2d(-38.39, -34.74))
//                    .build();
//
//            TrajectorySequence backdropPixelMiddle = drive.trajectorySequenceBuilder(dropPixelMiddle.end())
//                    .splineTo(new Vector2d(-55.98, -36.48), Math.toRadians(0.00))
//                    .build();
//
//            TrajectorySequence preParkMiddle = drive.trajectorySequenceBuilder(backdropPixelMiddle.end())
//                    .lineToConstantHeading(new Vector2d(-51.80, -12.10))
//                    .build();
//
//            TrajectorySequence parkMiddle = drive.trajectorySequenceBuilder(preParkMiddle.end())
//                    .lineToConstantHeading(new Vector2d(62.42, -10.53))
//                    .build();
//
//            CommandScheduler.getInstance().schedule(
//                    new SequentialCommandGroup(
//                            new WaitCommand(1000),
//                            new ParallelCommandGroup(
//                                    new TrajectorySequenceFollower(drive, firstGeneralMiddle),
//                                    new TapeDrop(robot)
//                            ),
//                            new TrajectorySequenceFollower(drive, dropPixelMiddle),
//                            new WaitCommand(350),
//                            new InstantCommand(() -> robot.claw.releaseLeft()),
//                            new WaitCommand(350),
//                            new ParallelCommandGroup(
//                                    new InstantCommand(() -> robot.a.armCoast()),
//                                    new InstantCommand(() -> robot.angle.intake()),
//                                    new TrajectorySequenceFollower(drive, backdropPixelMiddle)
//                            ),
//                            new WaitCommand(350),
//                            new InstantCommand(() -> robot.claw.grabLeft()),
//                            new WaitCommand(350),
//                            new InstantCommand(() -> robot.angle.outtake()),
//                            new TrajectorySequenceFollower(drive, preParkMiddle),
//                            new TrajectorySequenceFollower(drive, parkMiddle)
//                    )
//            );
//
//        } else if(elementPosition == 1) { //left
//            TrajectorySequence firstGeneralLeft = drive.trajectorySequenceBuilder(new Pose2d(-32.13, 63.64, Math.toRadians(90.00)))
//                    .lineToConstantHeading(new Vector2d(-38.39, 63.82))
//                    .build();
//
//            TrajectorySequence dropPixelLeft = drive.trajectorySequenceBuilder(firstGeneralLeft.end())
//                    .lineToSplineHeading(new Pose2d(-34.22, 31.95, Math.toRadians(-180.00)))
//                    .build();
//
//            TrajectorySequence preParkLeft = drive.trajectorySequenceBuilder(dropPixelLeft.end())
//                    .lineToSplineHeading(new Pose2d(-57.20, 11.75, Math.toRadians(0.00)))
//                    .build();
//
//            TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(preParkLeft.end())
//                    .lineToConstantHeading(new Vector2d(62.42, 11.58))
//                    .build();
//
//            CommandScheduler.getInstance().schedule(
//                    new SequentialCommandGroup(
//                            new WaitCommand(1000),
//                            new ParallelCommandGroup(
//                                    new TrajectorySequenceFollower(drive, firstGeneralLeft),
//                                    new TapeDrop(robot)
//                            ),
//                            new TrajectorySequenceFollower(drive, dropPixelLeft),
//                            new WaitCommand(350),
//                            new InstantCommand(() -> robot.claw.releaseLeft()),
//                            new WaitCommand(350),
//                            new ParallelCommandGroup(
//                                    new InstantCommand(() -> robot.a.armCoast()),
//                                    new InstantCommand(() -> robot.angle.intake()),
//                                    new TrajectorySequenceFollower(drive, preParkLeft)
//                            ),
//                            new WaitCommand(350),
//                            new InstantCommand(() -> robot.claw.grabLeft()),
//                            new WaitCommand(350),
//                            new InstantCommand(() -> robot.angle.outtake()),
//                            new TrajectorySequenceFollower(drive, preParkLeft),
//                            new TrajectorySequenceFollower(drive, parkLeft)
//                    )
//            );
//        }
//    }
//    @Override
//    public void loop() {
//        CommandScheduler.getInstance().run();
//        robot.a.loop();
//        drive.update();
//        robot.slides.loop();
//
//        double time = System.currentTimeMillis();
//        telemetry.addData("Loop: ", time - loop);
//        telemetry.addData("Arm Position: ", robot.a.getCachePos());
//        loop = time;
//
//        telemetry.update();
//    }
//
//    @Override
//    public void stop() {
//        CommandScheduler.getInstance().reset();
//    }
//
//    private void initTfod() {
//        TfodProcessor.Builder myTfodProcessorBuilder;
//        VisionPortal.Builder myVisionPortalBuilder;
//        VisionPortal myVisionPortal;
//
//        myTfodProcessorBuilder = new TfodProcessor.Builder();
//        myTfodProcessorBuilder.setModelFileName("REDTURKEY.tflite");
//        myTfodProcessorBuilder.setModelLabels(JavaUtil.createListWith("TSE"));
//        myTfodProcessorBuilder.setModelAspectRatio(16 / 9);
//        myTfodProcessor = myTfodProcessorBuilder.build();
//        myVisionPortalBuilder = new VisionPortal.Builder();
//        if (USE_WEBCAM) {
//            myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam"));
//        } else {
//            myVisionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
//        }
//        myVisionPortalBuilder.addProcessor(myTfodProcessor);
//        myVisionPortal = myVisionPortalBuilder.build();
//    }
//    private void telemetryTfod() {
//        boolean sensedElement;
//        List<Recognition> myTfodRecognitions;
//        Recognition myTfodRecognition;
//        float x;
//        float y;
//
//        sensedElement = false;
//        myTfodRecognitions = (List<Recognition>) myTfodProcessor.getRecognitions();
//        telemetry.addData("# Objects Detected", JavaUtil.listLength(myTfodRecognitions));
//        for (Recognition myTfodRecognition_item : myTfodRecognitions) {
//            myTfodRecognition = myTfodRecognition_item;
//            if (myTfodRecognition.getConfidence() * 100 > 70) {
//                telemetry.addLine("");
//                telemetry.addData("Image", myTfodRecognition.getLabel() + " (" + JavaUtil.formatNumber(myTfodRecognition.getConfidence() * 100, 0) + " % Conf.)");
//                x = (myTfodRecognition.getLeft() + myTfodRecognition.getRight()) / 2;
//                y = (myTfodRecognition.getTop() + myTfodRecognition.getBottom()) / 2;
//                telemetry.addData("- Position", JavaUtil.formatNumber(x, 0) + ", " + JavaUtil.formatNumber(y, 0));
//                telemetry.addData("- Size", JavaUtil.formatNumber(myTfodRecognition.getWidth(), 0) + " x " + JavaUtil.formatNumber(myTfodRecognition.getHeight(), 0));
//                if (x > 500) {
//                    sensedElement = true;
//                    elementPosition = 0;
//                    telemetry.addData("Team Element Detection: ", "Right");
//                } else if (x >= 190 && x <= 250) {
//                    sensedElement = true;
//                    elementPosition = 1;
//                    telemetry.addData("Team Element Detection: ", "Middle");
//                }
//            }
//        }
//        if (!sensedElement) {
//            elementPosition = 2;
//            telemetry.addData("Team Element Detection: ", "Left");
//        }
//    }
//    //0: right
//    //1: middle
//    //2: left
//}
