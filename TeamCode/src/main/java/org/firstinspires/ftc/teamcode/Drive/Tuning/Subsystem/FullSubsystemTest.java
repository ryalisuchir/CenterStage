package org.firstinspires.ftc.teamcode.Drive.Tuning.Subsystem;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.DriverCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.OuttakeCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.RestCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.TapeDropCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Utility.Hardware.RobotHardware;

@TeleOp
@Disabled
public class FullSubsystemTest extends CommandOpMode {
    private RobotHardware robot;
    private DriveSubsystem drive;
    private DriverCommand driveCommand;
    private GamepadEx gamepad2;
    double speed;
    @Override
    public void initialize() {
        robot = new RobotHardware(hardwareMap);

        drive = new DriveSubsystem(new SampleMecanumDrive(hardwareMap), false);

        robot.armSystem.armIntake();
        robot.angleOfArm.rest();
        speed = 0.5;

    }

    @Override
    public void run() {
        super.run();
        robot.armSystem.loop();
        robot.slidesSubsystem.loop();
        drive.update();

        driveCommand = new DriverCommand(
                drive, () -> -gamepad2.getLeftY(),
                gamepad2::getLeftX, gamepad2::getRightX
        );

        schedule(
                driveCommand
        );

        boolean x = gamepad1.x;
        if (x) {
            schedule(
                    new InstantCommand(() -> robot.claw.grabBoth())
            );
        }
        boolean y = gamepad1.y;
        if (y) {
            schedule(
                    new InstantCommand(() -> robot.claw.releaseBoth())
            );
        }
        boolean a = gamepad1.a;
        if (a) {
            schedule(
                    new IntakeCommand(robot)
            );
        }
        boolean b = gamepad1.b;
        if (b) {
            schedule (
                    new OuttakeCommand(robot)
            );
        }
        boolean gamepadUp = gamepad1.dpad_up;
        if (gamepadUp) {
            schedule(
                    new RestCommand(robot)
            );
        }

        boolean gamepadDown = gamepad1.dpad_down;
        if (gamepadDown) {
            schedule(
                    new TapeDropCommand(robot)
            );
        }

    }
}
