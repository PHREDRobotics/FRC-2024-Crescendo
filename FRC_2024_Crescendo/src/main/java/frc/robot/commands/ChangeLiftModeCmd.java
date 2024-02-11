package frc.robot.commands;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LiftSubsystem;

public class ChangeLiftModeCmd extends Command {
    
    IdleMode m_mode;
    LiftSubsystem m_lift_subsystem;

    public ChangeLiftModeCmd(IdleMode mode, LiftSubsystem liftSubsystem) {
        this.m_mode = mode;
        this.m_lift_subsystem = liftSubsystem;
    }

    @Override
    public void initialize() {
        this.m_lift_subsystem.changeLiftMode(m_mode);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
