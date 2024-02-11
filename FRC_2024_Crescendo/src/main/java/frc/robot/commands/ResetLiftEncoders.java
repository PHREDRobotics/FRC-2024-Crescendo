package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LiftSubsystem;

/**
 * Command to reset the encoders on the lift
 */
public class ResetLiftEncoders extends Command {
    private LiftSubsystem m_lift_subsystem;

    public ResetLiftEncoders(LiftSubsystem liftSubsystem) {
        m_lift_subsystem = liftSubsystem;
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        m_lift_subsystem.resetEncoders();
    }
}
