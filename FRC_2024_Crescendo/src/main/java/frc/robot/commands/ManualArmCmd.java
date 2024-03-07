package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;

/**
 * Command for manually moving the arm
 */
public class ManualArmCmd extends Command {

  private DoubleSupplier armPower;
  private ArmSubsystem armSubsystem;

  /**
   * Command to manually control the arm
   * 
   * @param ArmPower     power for the arm motor
   * @param ArmSubsystem the Arm Subsystem
   */
  public ManualArmCmd(DoubleSupplier ArmPower, ArmSubsystem ArmSubsystem) {
    this.armPower = ArmPower;
    this.armSubsystem = ArmSubsystem;

    addRequirements(armSubsystem);
  }

  @Override
  public void initialize() {
    armSubsystem.disable();
    // arm_subsystem.resetEncoders();
  }

  @Override
  public void execute() {
    double power = this.armPower.getAsDouble();
    if (Math.abs(power) < OIConstants.kDeadband) {
      power = 0;
    }
    // SQUARE IT!
    power = power * Math.abs(power);
    this.armSubsystem.setRawPower(power);
  }

   // Called once the command ends or is interrupted.
   @Override
   public void end(boolean interrupted) {
    
   }
 
   // Returns true when the command should end.
   @Override
   public boolean isFinished() {
     return false;
   }


}
