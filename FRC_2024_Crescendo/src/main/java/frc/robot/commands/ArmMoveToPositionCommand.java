package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

/**
 * A command for moving the arm
 */
public class ArmMoveToPositionCommand extends Command {
  private final ArmSubsystem m_armSubsystem;
  private final double m_armTargetPosition;

  /**
   * Operate the arm subsystem
   * 
   * @param armTargetPosition the Position to which the arm should move (Radians)
   * @param armSubsystem
   */
  public ArmMoveToPositionCommand(double armTargetPosition, ArmSubsystem armSubsystem) {
    m_armTargetPosition = armTargetPosition;
    m_armSubsystem = armSubsystem;
    addRequirements(m_armSubsystem);
  }

  @Override
  public void initialize() {
    m_armSubsystem.enable();
    m_armSubsystem.moveToPosition(m_armTargetPosition);
  }

  @Override
  public void execute() {
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted == false) {
      // m_armSubsystem.moveToPosition(Constants.ArmConstants.kArmPickup);
    } else {
      m_armSubsystem.disable();
    }
  }
}