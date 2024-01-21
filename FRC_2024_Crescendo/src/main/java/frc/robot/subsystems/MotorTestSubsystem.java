package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.TestConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class MotorTestSubsystem extends SubsystemBase {

  private CANSparkMax driveSparkMax = new CANSparkMax(TestConstants.kTestMotorCanId, MotorType.kBrushless);
  private CANSparkMax driveSparkMaxTwo = new CANSparkMax(TestConstants.kTestMotorCanIdTwo, MotorType.kBrushless);

  public MotorTestSubsystem() {
    super();
  }

  public void drive(double speed) {
      driveSparkMax.set(speedConvert(speed));
      driveSparkMaxTwo.set(-speedConvert(speed));
    }
  

  @Override
  public void periodic() {
  }

  private double speedConvert(double inSpeed) {

    if (inSpeed < 0.2 && inSpeed > -0.2) {
      return 0.0;
    }

    return inSpeed;
  }

}
