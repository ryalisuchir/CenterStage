package org.firstinspires.ftc.teamcode.drive.tests.hardware;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commandbase.command.GrabBothCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.OuttakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.ReleaseBothCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.RestCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.TapeDropperCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

@TeleOp
public class SubsystemTest extends CommandOpMode {
    private Robot robot;
    double speed;
    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);

        robot.a.armIntake();
        robot.angle.rest();
        speed = 0.5;
    }

    @Override
    public void run() {
        super.run();
        robot.a.loop();

        boolean x = gamepad1.x;
        if (x) {
            schedule(
                    new GrabBothCommand(robot)
            );
        }
        boolean y = gamepad1.y;
        if (y) {
            schedule(
                    new ReleaseBothCommand(robot)
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
                    new TapeDropperCommand(robot)
            );
        }

    }
}
