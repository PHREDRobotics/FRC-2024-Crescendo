package frc.robot.compositions;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.commands.ArmMotor;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.IntakeCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;


public class LowerIntakeComposition extends ParallelCommandGroup {
  private final ArmSubsystem m_subsystem;
  private final IntakeSubsystem n_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param m_subsystem The subsystem used by this command.
   * @param n_subsystem
   */
  public LowerIntakeComposition(ArmSubsystem arm, IntakeSubsystem intake) {
    m_subsystem = arm;
    n_subsystem = intake;

    addRequirements(arm);
    addRequirements(intake);
    addCommands(
        // intake
        new IntakeCommand(n_subsystem),

        new ArmMotor(m_subsystem)//,

    // Drive backward the specified distance
    // new DriveDistance(
    // AutoConstants.kAutoBackupDistanceInches, -AutoConstants.kAutoDriveSpeed,
    // drive)
    );
  }

  // // Called when the command is initially scheduled.
  // @Override
  // public void initialize() {
  // }

  // // Called every time the scheduler runs while the command is scheduled.
  // @Override
  // public void execute() {
  // }

  // // Called once the command ends or is interrupted.
  // @Override
  // public void end(boolean interrupted) {
  // }

  // // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   return false;
  // }

}
