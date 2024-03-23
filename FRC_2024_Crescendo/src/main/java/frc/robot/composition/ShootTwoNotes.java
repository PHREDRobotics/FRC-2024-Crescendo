// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.composition;

import frc.robot.subsystems.*;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.*;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class ShootTwoNotes extends SequentialCommandGroup {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final ArmSubsystem m_ArmSubsystem;
  private final IntakeSubsystem m_IntakeSubsystem;
  private final ShooterSubsystem m_ShoooterSubsystem;
  private final SwerveSubsystem m_SwerveSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShootTwoNotes(ArmSubsystem arm, IntakeSubsystem intake, ShooterSubsystem shoooter,
      SwerveSubsystem swerve) {
    m_ArmSubsystem = arm;
    m_IntakeSubsystem = intake;
    m_ShoooterSubsystem = shoooter;
    m_SwerveSubsystem = swerve;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
    addRequirements(intake);
    addRequirements(shoooter);
    addRequirements(swerve);

    addCommands(
        // Resets odometery and heading
        new InstantCommand(() -> m_SwerveSubsystem.zeroHeading()),
        // automatically reset the arm using the limit switch
        new AutoResetArmEncoder(m_ArmSubsystem),
        // outtake and shoot a note
        new ParallelCommandGroup(
            new ShooterCommand(m_ShoooterSubsystem),
            new OuttakeCommand(m_IntakeSubsystem)),
        // intakes, drives forwards, and lowers the arm
        new ParallelDeadlineGroup(
            new IntakeCommand(m_IntakeSubsystem),
            new GoToPose2d(swerve, new Translation2d(2, 0.0)),
            new ArmMoveToPositionCommand(ArmConstants.kArmPickup, arm)),
        // drives backwards, and raises the arm
        new ParallelDeadlineGroup(
            new GoToPose2d(swerve, new Translation2d(0.0, 0.0)),
            new AutoResetArmEncoder(arm)),
        // outtake and shoot a note
        new ParallelCommandGroup(
            new ShooterCommand(m_ShoooterSubsystem),
            new OuttakeCommand(m_IntakeSubsystem)),
        new GoToPose2d(swerve, new Translation2d(3.0, 0.0)));
  }
}
