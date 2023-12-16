package org.firstinspires.ftc.teamcode.common.commandbase.command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystems.ClawSubsystem;

public class GrabBoth extends CommandBase {

    private final ClawSubsystem m_clawSubsystem;

    public GrabBoth(ClawSubsystem subsystem) {
        m_clawSubsystem = subsystem;
        addRequirements(m_clawSubsystem);
    }

    @Override
    public void initialize() {
        m_clawSubsystem.grabBoth();
    }
    @Override
    public boolean isFinished() {
        return true;
    }

}