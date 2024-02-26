package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LiftSubsystem;

public class ExtendRight extends Command {
    
    public LiftSubsystem m_lift_subsystem;

    public ExtendRight(LiftSubsystem liftSubsystem) {
        m_lift_subsystem = liftSubsystem;
    }

    @Override
    public void execute() {
        m_lift_subsystem.extendRightLift();
    }

    // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_lift_subsystem.stopRightLift();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_lift_subsystem.isRightDone();
  }

    
}
