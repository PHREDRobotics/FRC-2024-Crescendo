package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TestConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {

  private CANSparkMax LeftMotor = new CANSparkMax(TestConstants.kTestMotorCanId, MotorType.kBrushless);
  private CANSparkMax RightMotor = new CANSparkMax(TestConstants.kTestMotorCanIdTwo, MotorType.kBrushless);

  public IntakeSubsystem() {
    super();
  }

  public Command Intake() {
    boolean beamBroken = false;
    if (beamBroken) {
      return runOnce(
          () -> {
            LeftMotor.set(0);
            RightMotor.set(0);
          });
    } else {
      return runOnce(
          () -> {
            LeftMotor.set(speedConvert(1));
            RightMotor.set(-speedConvert(1));
          });
    }

  }

  public void Outtake() {
    LeftMotor.set(speedConvert(-1));
    RightMotor.set(-speedConvert(-1));
  }

  private double speedConvert(double inSpeed) {

    if (inSpeed < 0.2 && inSpeed > -0.2) {
      return 0.0;
    }

    return inSpeed;
  }

}
