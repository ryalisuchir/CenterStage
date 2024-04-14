package org.firstinspires.ftc.teamcode.Drive.OpModes.TeleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.PlusTwoBlueStackCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.RestCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.SuperHighOuttakeCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.TwoPixelDropCommand;
import org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands.xPerimentalGrab;
import org.firstinspires.ftc.teamcode.Utility.Hardware.RobotHardware;

@TeleOp
public class Subsystems extends CommandOpMode {
    private RobotHardware robot;
    double speed;
    @Override
    public void initialize() {
        robot = new RobotHardware(hardwareMap);
        robot.armSystem.armCoast();
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
                    new TwoPixelDropCommand(robot)
            );
        }

        boolean b = gamepad1.b;
        if (b) {
            schedule(
                    new RestCommand(robot)
            );
        }

        boolean dpad_up = gamepad1.dpad_up;
        if (dpad_up) {
            schedule(
                    new SuperHighOuttakeCommand(robot)
            );
        }

        boolean dpad_down = gamepad1.dpad_down;
        if (dpad_down) {
            schedule(
                    new PlusTwoBlueStackCommand(robot)
            );
        }

        boolean dpad_left = gamepad1.dpad_left;
        if (dpad_left) {
            schedule(
                    new xPerimentalGrab(robot)
            );
        }

    }
}