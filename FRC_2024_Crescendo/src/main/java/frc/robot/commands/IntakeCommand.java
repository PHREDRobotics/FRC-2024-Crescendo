// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkLowLevel.MotorType;

/** An Intake command that uses an Intake subsystem. */
public class IntakeCommand extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final IntakeSubsystem m_subsystem;

  private CANSparkMax m_upMotor;
  private CANSparkMax m_downMotor;

  private SparkLimitSwitch m_forwardLimitSwitch;

  /**
   * Creates a new IntakeCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntakeCommand(IntakeSubsystem subsystem) {
    m_subsystem = subsystem;
    m_upMotor = m_subsystem.m_upMotor;
    m_downMotor = m_subsystem.m_downMotor;
    m_forwardLimitSwitch = m_subsystem.m_forwardLimit;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_upMotor.set(Constants.TestConstants.kTestIntakeSpeed);
    m_downMotor.set(-Constants.TestConstants.kTestIntakeSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_forwardLimitSwitch.isPressed()) {
      m_upMotor.set(m_subsystem.speedConvert(0));
      m_downMotor.set(m_subsystem.speedConvert(0));
    } else {
      m_upMotor.set(Constants.TestConstants.kTestIntakeSpeed);
      m_downMotor.set(-Constants.TestConstants.kTestIntakeSpeed);
    }
  }
  // down motor speed = -up motor speed

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_upMotor.set(m_subsystem.speedConvert(0));
    m_downMotor.set(m_subsystem.speedConvert(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
