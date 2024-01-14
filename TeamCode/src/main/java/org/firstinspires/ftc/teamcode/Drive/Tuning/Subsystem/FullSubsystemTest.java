package org.firstinspires.ftc.teamcode.Drive.Tuning.Subsystem;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.OuttakeCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.RestCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.TapeDropCommand;
import org.firstinspires.ftc.teamcode.Utility.Hardware.RobotHardware;

@TeleOp

public class FullSubsystemTest extends CommandOpMode {
    private RobotHardware robot;
    private GamepadEx gamepad2;
    double speed;
    @Override
    public void initialize() {
        robot = new RobotHardware(hardwareMap);

        robot.armSystem.armIntake();
        robot.angleOfArm.rest();
        speed = 0.5;

    }

    @Override
    public void run() {
        super.run();
        robot.armSystem.loop();
        robot.slidesSubsystem.loop();

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
            schedule(
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