package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TestConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;

import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj.Timer;
import javax.print.attribute.standard.RequestingUserName;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {
  private final Timer m_timer = new Timer();

  public CANSparkMax m_upMotor = new CANSparkMax(TestConstants.kTestMotorCanId, MotorType.kBrushless);
  public CANSparkMax m_downMotor = new CANSparkMax(TestConstants.kTestMotorCanIdTwo, MotorType.kBrushless);
  public SparkLimitSwitch m_forwardLimit = m_upMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

  // public SparkLimitSwitch m_forwardLimit =
  // m_upMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);

  public IntakeSubsystem() {
    super();

    // m_forwardLimit.enableLimitSwitch(false);
  }

  // public Command Intake() {
  // boolean beamBroken = false;
  // if (beamBroken) {
  // return runOnce(
  // () -> {
  // m_upMotor.set(0);
  // m_downMotor.set(0);
  // });
  // } else {
  // return runOnce(
  // () -> {
  // m_upMotor.set(speedConvert(1));
  // m_downMotor.set(-speedConvert(1));
  // });
  // }

  // }

  public void Outtake() {
    m_upMotor.set(speedConvert(-1));
    m_downMotor.set(speedConvert(-1));
    m_timer.start();
  }

  public double speedConvert(double inSpeed) {

    if (inSpeed < 0.2 && inSpeed > -0.2) {
      return 0.0;
    }

    return inSpeed;
  }

  public void stopIntake() {
    m_upMotor.set(0);
    m_downMotor.set(0);
  }

  public void pickUpNote() {
    // This will stop when the beam in our beam break sensor is broken

    m_upMotor.set(Constants.TestConstants.kTestIntakeSpeed);
    m_downMotor.set(Constants.TestConstants.kTestIntakeSpeed);
  }

  /*
   * public void dropNote() {
   * // reset the timer
   * timer.reset();
   * 
   * 
   * m_upMotor.set(-Constants.TestConstants.kTestIntakeSpeed);
   * m_downMotor.set(-Constants.TestConstants.kTestIntakeSpeed);
   * }
   */
  public boolean isNoteLoaded() {

    return m_forwardLimit.isPressed();
  }

  public boolean outtakeIsTimeDone() {
    return m_timer.hasElapsed(Constants.IntakeConstants.kOuttakeTime);

  }

  public void ejectToShooter() {
    // This will be slower than ejectToAmp

  }

  public void ejectToAmp() {
    // This will be faster ten ejectToShooter

  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Pressed?", isNoteLoaded());
    // This method will be called once per scheduler run
    // We will have a pull in fast and slow and a push out fast and slow
    // When we pull in we will use the beam break sensor to stop the motor

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
