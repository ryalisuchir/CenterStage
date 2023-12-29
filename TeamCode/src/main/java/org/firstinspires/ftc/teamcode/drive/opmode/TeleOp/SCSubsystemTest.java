package org.firstinspires.ftc.teamcode.drive.opmode.TeleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.commandbase.command.GrabBothCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.OuttakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.ReleaseBothCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.RestCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.TapeDropperCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.squeakycleancommands.SCIntakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.squeakycleancommands.SCOuttakeCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.squeakycleancommands.SCRestCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.squeakycleancommands.SCTapeDropperCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Robot;

@TeleOp
public class SCSubsystemTest extends CommandOpMode {
    private Robot robot;
    double speed;
    @Override
    public void initialize() {
        robot = new Robot(hardwareMap);

        robot.angle.rest();
        speed = 0.5;
    }

    @Override
    public void run() {
        super.run();
        robot.sca.armLooper();

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
                    new SCIntakeCommand(robot)
            );
        }
        boolean b = gamepad1.b;
        if (b) {
            schedule (
                    new SCOuttakeCommand(robot)
            );
        }
        boolean gamepadUp = gamepad1.dpad_up;
        if (gamepadUp) {
            schedule(
                    new SCRestCommand(robot)
            );
        }

        boolean gamepadDown = gamepad1.dpad_down;
        if (gamepadDown) {
            schedule(
                    new SCTapeDropperCommand(robot)
            );
        }

    }
}
