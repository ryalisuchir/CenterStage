//package org.firstinspires.ftc.teamcode.common.commandbase.command;
//
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.ParallelCommandGroup;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//import com.arcrobotics.ftclib.command.WaitUntilCommand;
//
//import org.firstinspires.ftc.teamcode.common.hardware.Robot;
//
////command to change angle of arm and then open right claw
//public class OuttakePosition extends SequentialCommandGroup {
//    public OuttakePosition(Robot robot) {
//        super(
//                new ParallelCommandGroup(
//                        new InstantCommand(() -> robot.a.armOuttake()),
//                        new InstantCommand(() -> robot.angle.outtake())
//                )
//        );
//    }
//}