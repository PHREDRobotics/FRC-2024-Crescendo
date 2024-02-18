package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LiftSubsystem;

public class ExtendLeft extends Command {
    
    public LiftSubsystem m_lift_subsystem;

    public ExtendLeft(LiftSubsystem liftSubsystem) {
        m_lift_subsystem = liftSubsystem;
    }

    @Override
    public void execute() {
        m_lift_subsystem.extendLeftLift();
    }

    // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_lift_subsystem.stopLeftLift();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_lift_subsystem.isLeftDone();
  }

    
}
