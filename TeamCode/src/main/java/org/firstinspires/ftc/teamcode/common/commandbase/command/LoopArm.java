//package org.firstinspires.ftc.teamcode.common.commandbase.command;
//
//import com.arcrobotics.ftclib.command.CommandBase;
//
//import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.ArmSubsystem;
//import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.ClawSubsystem;
//
//public class LoopArm extends CommandBase {
//
//    private final ArmSubsystem m_armSubsystem;
//
//    public LoopArm(ArmSubsystem subsystem) {
//        m_armSubsystem = subsystem;
//        addRequirements(m_armSubsystem);
//    }
//
//    @Override
//    public void initialize() {
//        m_armSubsystem.loop();
//    }
//    @Override
//    public boolean isFinished() {
//        return true;
//    }
//
//}