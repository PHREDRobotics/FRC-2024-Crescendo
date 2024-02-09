package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LiftSubsystem;

public class ExtendLift extends Command {
    
    public LiftSubsystem m_lift_subsystem;

    public ExtendLift(LiftSubsystem liftSubsystem) {
        m_lift_subsystem = liftSubsystem;
    }

    @Override
    public void execute() {
        m_lift_subsystem.extendLeftLift();
    }
}
