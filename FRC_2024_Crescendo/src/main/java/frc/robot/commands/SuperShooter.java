// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import javax.sound.midi.Sequence;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. 
 * @param <m_Intake>*/
public class SuperShooter<m_Intake> extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final IntakeSubsystem m_Intake;
  private final ShooterSubsystem m_Shooter;

  /**
   * Creates a new ExampleCommand.
 * @param <m_Shooter>
   *
   * @param subsystem The subsystem used by this command.
   */
  public <m_Shooter> SuperShooter(m_Shooter Shooter, m_Intake Intake) {
    m_Shooter = (ShooterSubsystem) Shooter;
    m_Intake = (IntakeSubsystem) Intake;
    addRequirements(m_Shooter);
    addRequirements(m_Intake);

    //possibly fix above???
    
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands( // CODE COMMANDS IN HERE

//move the arm to the shooter position Andys job
//run the outtake command
//run the shooter command




       /*  new ResetLifterEncoder(m_Lifter)
            .andThen(new PlaceHighLevel(m_Lifter))
            .andThen(new ExtendNearArmExtension(m_ArmPneumatics))
            .andThen(new Place(m_Grabber)) */

    );
  }

  @Override
  public boolean runsWhenDisabled() {
    return false;
  }
}
