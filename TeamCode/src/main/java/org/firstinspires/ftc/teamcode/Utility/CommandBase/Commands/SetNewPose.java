package org.firstinspires.ftc.teamcode.Utility.CommandBase.Commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Utility.Hardware.RobotHardware;

public class SetNewPose extends SequentialCommandGroup {


    public SetNewPose(RobotHardware robot) {

        robot.driveSubsystem.setPoseEstimate(new Pose2d(57, -32, 0));

    }

}
//robot.armSystem.armCoast()),