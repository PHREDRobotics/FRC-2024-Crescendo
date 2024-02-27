// package frc.robot.compositions;

// import frc.robot.subsystems.ArmSubsystem;
// import frc.robot.subsystems.IntakeSubsystem;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// public class PickUpComposition extends SequentialCommandGroup {
//   private final ArmSubsystem m_subsystem;
//   private final IntakeSubsystem n_subsystem;

//   /**
//    * Creates a new ExampleCommand.
//    *
//    * @param m_subsystem The subsystem used by this command.
//    * @param n_subsystem
//    */
//   public PickUpComposition(ArmSubsystem arm, IntakeSubsystem intake) {
//     m_subsystem = arm;
//     n_subsystem = intake;

//     addRequirements(arm);
//     addRequirements(intake);
//     addCommands(
//         // lower and intake
//         new LowerIntakeComposition(
//             m_subsystem, n_subsystem)// ,

//     // Release the hatch
//     // new ReleaseHatch(hatch),

//     // Drive backward the specified distance
//     // new DriveDistance(
//     // AutoConstants.kAutoBackupDistanceInches, -AutoConstants.kAutoDriveSpeed,
//     // drive)
//     );
//   }

//   // // Called when the command is initially scheduled.
//   // @Override
//   // public void initialize() {
//   // }

//   // // Called every time the scheduler runs while the command is scheduled.
//   // @Override
//   // public void execute() {
//   // }

//   // // Called once the command ends or is interrupted.
//   // @Override
//   // public void end(boolean interrupted) {
//   // }

//   // // Returns true when the command should end.
//   // @Override
//   // public boolean isFinished() {
//   //   return false;
//   // }
// }
