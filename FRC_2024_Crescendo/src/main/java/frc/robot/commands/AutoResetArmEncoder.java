// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AutoResetArmEncoder extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final ArmSubsystem m_subsystem;
  private boolean limit_switch_on = false;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoResetArmEncoder(ArmSubsystem subsystem, boolean limitSwitchOn) {
    m_subsystem = subsystem;
    limit_switch_on = limitSwitchOn;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setRawPower(-0.2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      m_subsystem.moveToPosition(Constants.ArmConstants.kArmMid);
    } else {
      m_subsystem.resetEncoders();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (limit_switch_on) {
      return true;
    } else {
      return false;
    }
  }
}
