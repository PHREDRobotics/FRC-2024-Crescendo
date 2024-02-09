package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LiftSubsystem;

public class RetractLift extends Command {
    
    public LiftSubsystem m_lift_subsystem;

    public RetractLift(LiftSubsystem liftSubsystem) {
        m_lift_subsystem = liftSubsystem;
    }

    @Override
    public void execute() {
        m_lift_subsystem.retractLeftLift();
    }
}
