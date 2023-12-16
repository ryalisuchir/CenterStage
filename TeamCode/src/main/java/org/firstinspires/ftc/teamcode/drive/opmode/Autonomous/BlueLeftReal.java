package org.firstinspires.ftc.teamcode.drive.opmode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.AngleSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.Drive;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.Slides;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous
public class BlueLeftReal extends OpMode {
    private ElapsedTime time_since_start;
    public DcMotorEx leftFront, rightFront, leftRear, rightRear, linear_1, linear_2, arm;
    public Servo dump, claw1, claw2;
    public VoltageSensor batteryVoltageSensor;

    public AngleSubsystem angle;

    public ArmSubsystem a;
    public ClawSubsystem claw;
    public Slides slides;

    public Drive drive;
    public static double MAX_CURRENT = 15;
    TfodProcessor myTfodProcessor;
    int elementPosition;
    boolean USE_WEBCAM;
    private double loop;
    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        //brake
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //reverse
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        //set external motors
        linear_1 = hardwareMap.get(DcMotorEx.class, "linear_1");
        linear_2 = hardwareMap.get(DcMotorEx.class, "linear_2");
        arm = hardwareMap.get(DcMotorEx.class, "arm");

        linear_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linear_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        linear_2.setDirection(DcMotor.Direction.REVERSE);
        linear_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linear_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //servos
        Servo dump = hardwareMap.get(Servo.class, "dump");
        Servo claw1 = hardwareMap.get(Servo.class, "claw");
        Servo claw2 = hardwareMap.get(Servo.class, "claw1");


        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        //setting subsystems
        a = new ArmSubsystem(arm, batteryVoltageSensor);
        claw = new ClawSubsystem(hardwareMap, "claw", "claw1");
        angle = new AngleSubsystem(hardwareMap, "dump");
        drive = new Drive(new SampleMecanumDrive(hardwareMap), false);

        CommandScheduler.getInstance().registerSubsystem(a, claw, angle, drive);

        CommandScheduler.getInstance().registerSubsystem(a);
        CommandScheduler.getInstance().registerSubsystem(claw);
        CommandScheduler.getInstance().registerSubsystem(angle);
        CommandScheduler.getInstance().registerSubsystem(drive);

        USE_WEBCAM = true;
        initTfod();
        telemetry.addData("D.S. Preview: ", "Ready for BlueLeft (Backdrop Side)");
        telemetry.addData("Running: ", "2 pixel autonomous. All subsystems will run.");
        telemetry.update();

        claw.grabBoth();

    }

    @Override
    public void init_loop() {
        telemetryTfod();
        CommandScheduler.getInstance().run();
        a.loop();
    }

    public void start() {
        time_since_start = new ElapsedTime();
        if(elementPosition == 0) { //right

            TrajectorySequence dropPixelRight = drive.trajectorySequenceBuilder(new Pose2d(18.89, 66.78, Math.toRadians(-90.00)))
                    .splineTo(
                            new Vector2d(11.23, 35.78), Math.toRadians(205.71),
                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .build();

            TrajectorySequence backdropPixelRight = drive.trajectorySequenceBuilder(dropPixelRight.end())
                    .lineToConstantHeading(
                            new Vector2d(22.55, 46.93),
                            SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .splineTo(
                            new Vector2d(47.97, 24.29), Math.toRadians(0.00),
                            SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .build();

            TrajectorySequence backupRight = drive.trajectorySequenceBuilder(backdropPixelRight.end())
                    .lineToConstantHeading(
                            new Vector2d(38.57, 24.64),
                            SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .build();

            TrajectorySequence parkRight = drive.trajectorySequenceBuilder(backupRight.end())
                    .lineToConstantHeading(
                            new Vector2d(48.49, 59.99),
                            SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .lineToConstantHeading(
                            new Vector2d(61.38, 60.33),
                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .build();


            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new WaitCommand(500),
                            new InstantCommand(() -> drive.followTrajectorySequence(dropPixelRight)),
                            new WaitCommand(1000),
                            new ParallelCommandGroup(
                                    new InstantCommand(() -> a.armOuttake()),
                                    new InstantCommand(() -> angle.outtake()),
                                    new InstantCommand(() -> drive.followTrajectorySequence(backdropPixelRight))
                            ),
                            new WaitCommand(1000),
                            new InstantCommand(() -> claw.releaseRight()),
                            new WaitCommand(1000),
                            new InstantCommand(() -> drive.followTrajectorySequence(backupRight)),
                            new ParallelCommandGroup(
                                    new InstantCommand(() -> claw.grabRight()),
                                    new InstantCommand(() -> angle.intake()),
                                    new InstantCommand(() -> a.armCoast())
                            ),
                            new InstantCommand(() -> drive.followTrajectorySequence(parkRight))
                    )
            );
        } else if(elementPosition == 1) { //middle //MIDDLE MIDDLE MIDDLE RIGHT HERE SUCHIR PAY ATTENTION SUCHIR RIGHT HERE
            TrajectorySequence dropPixelMiddle = drive.trajectorySequenceBuilder(new Pose2d(18.89, 66.78, Math.toRadians(-90.00)))
                    .lineToConstantHeading(
                            new Vector2d(15.06, 10.36),
                            SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .build();

            TrajectorySequence backdropPixelMiddle = drive.trajectorySequenceBuilder(dropPixelMiddle.end())
                    .lineToConstantHeading(
                            new Vector2d(15.41, 27.95),
                            SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .splineTo(
                            new Vector2d(47.80, 37.87), Math.toRadians(0.00),
                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .build();

            TrajectorySequence backupMiddle = drive.trajectorySequenceBuilder(backdropPixelMiddle.end())
                    .lineToConstantHeading(
                            new Vector2d(41.18, 37.70),
                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .build();

            TrajectorySequence parkMiddle = drive.trajectorySequenceBuilder(backupMiddle.end())
                    .lineToConstantHeading(
                            new Vector2d(62.07, 60.51),
                            SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .lineToConstantHeading(
                            new Vector2d(48.15, 60.68),
                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .build();


            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new WaitCommand(500),
                            new InstantCommand(() -> drive.followTrajectorySequence(dropPixelMiddle)),
                            new WaitCommand(1000),
                            new ParallelCommandGroup(
                                    new InstantCommand(() -> a.armOuttake()),
                                    new InstantCommand(() -> angle.outtake()),
                                    new InstantCommand(() -> drive.followTrajectorySequence(backdropPixelMiddle))
                            ),
                            new WaitCommand(1000),
                            new InstantCommand(() -> claw.releaseRight()),
                            new WaitCommand(1000),
                            new InstantCommand(() -> drive.followTrajectorySequence(backupMiddle)),
                            new ParallelCommandGroup(
                                    new InstantCommand(() -> claw.grabRight()),
                                    new InstantCommand(() -> angle.intake()),
                                    new InstantCommand(() -> a.armCoast())
                            ),
                            new InstantCommand(() -> drive.followTrajectorySequence(parkMiddle))
                    )
            );

        } else if(elementPosition == 1) { //left
            TrajectorySequence dropPixelLeft = drive.trajectorySequenceBuilder(new Pose2d(18.89, 66.78, Math.toRadians(-90.00)))
                    .lineToConstantHeading(
                            new Vector2d(28.99, 39.61),
                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .build();

            TrajectorySequence backdropPixelLeft = drive.trajectorySequenceBuilder(dropPixelLeft.end())
                    .lineToConstantHeading(
                            new Vector2d(28.12, 53.89),
                            SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .splineTo(
                            new Vector2d(47.80, 37.87), Math.toRadians(0.00),
                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .build();

            TrajectorySequence backupLeft = drive.trajectorySequenceBuilder(backdropPixelLeft.end())
                    .lineToConstantHeading(
                            new Vector2d(41.18, 37.70),
                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .build();

            TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(backupLeft.end())
                    .lineToConstantHeading(
                            new Vector2d(62.07, 60.51),
                            SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .lineToConstantHeading(
                            new Vector2d(48.15, 60.68),
                            SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .build();


            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new WaitCommand(500),
                            new InstantCommand(() -> drive.followTrajectorySequence(dropPixelLeft)),
                            new WaitCommand(1000),
                            new ParallelCommandGroup(
                                    new InstantCommand(() -> a.armOuttake()),
                                    new InstantCommand(() -> angle.outtake()),
                                    new InstantCommand(() -> drive.followTrajectorySequence(backdropPixelLeft))
                            ),
                            new WaitCommand(1000),
                            new InstantCommand(() -> claw.releaseRight()),
                            new WaitCommand(1000),
                            new InstantCommand(() -> drive.followTrajectorySequence(backupLeft)),
                            new ParallelCommandGroup(
                                    new InstantCommand(() -> claw.grabRight()),
                                    new InstantCommand(() -> angle.intake()),
                                    new InstantCommand(() -> a.armCoast())
                            ),
                            new InstantCommand(() -> drive.followTrajectorySequence(parkLeft))
                    )
            );
        }
    }
    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        a.loop();
        drive.update();

        double time = System.currentTimeMillis();
        telemetry.addData("Loop: ", time - loop);
        telemetry.addData("Arm Position: ", a.getCachePos());
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
                    elementPosition = 0; // right
                    telemetry.addData("Team Element Detection: ", "Right");
                } else if (x >= 190 && x <= 250) {
                    sensedElement = true;
                    elementPosition = 1; // middle
                    telemetry.addData("Team Element Detection: ", "Middle");
                }
            }
        }
        if (!sensedElement) {
            elementPosition = 2; // left
            telemetry.addData("Team Element Detection: ", "Left");
        }
    }
    //0: right
    //1: middle
    //2: left
}