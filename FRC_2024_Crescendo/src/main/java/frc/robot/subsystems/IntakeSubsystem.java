package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
<<<<<<< HEAD
import frc.robot.Constants;
import frc.robot.Constants.GrabberConstants;
=======
import frc.robot.Constants.IntakeConstants;
>>>>>>> 626e5c332d4f3aec8f29559f5e3a8eb5961e7253
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
  private static final Timer m_timer = new Timer();

<<<<<<< HEAD
  public CANSparkMax m_upMotor = new CANSparkMax(TestConstants.kTestMotorCanId, MotorType.kBrushless);
  public CANSparkMax m_downMotor = new CANSparkMax(TestConstants.kTestMotorCanIdTwo, MotorType.kBrushless);
  public SparkLimitSwitch m_forwardLimit = m_upMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

  public double m_outtakeSpeed = TestConstants.kTestOuttakeSpeed;
  public double m_intakeSpeed = TestConstants.kTestIntakeSpeed;

  // public SparkLimitSwitch m_forwardLimit =
  // m_upMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
=======
  private CANSparkMax LeftMotor = new CANSparkMax(IntakeConstants.kLeftIntakeControllerPort, MotorType.kBrushless);
  private CANSparkMax RightMotor = new CANSparkMax(IntakeConstants.kRightIntakeControllerPort, MotorType.kBrushless);
>>>>>>> 626e5c332d4f3aec8f29559f5e3a8eb5961e7253

  public IntakeSubsystem() {
    super();
    SmartDashboard.putNumber("Outtake Speed", m_outtakeSpeed);
    SmartDashboard.putNumber("Intake Speed", m_intakeSpeed);
    // m_forwardLimit.enableLimitSwitch(false);
  }

<<<<<<< HEAD
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
    m_timer.reset();
    m_upMotor.set(speedConvert(-m_outtakeSpeed));
    m_downMotor.set(speedConvert(m_outtakeSpeed));
    m_timer.start();
=======
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
>>>>>>> 626e5c332d4f3aec8f29559f5e3a8eb5961e7253
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

    m_upMotor.set(m_intakeSpeed);
    m_downMotor.set(-m_intakeSpeed);
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

    return m_forwardLimit.isPressed() || SmartDashboard.getBoolean("Manual Override Press", false);
  }

  public static boolean outtakeIsTimeDone() {
    return m_timer.hasElapsed(Constants.GrabberConstants.kOuttakeTime);

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
    SmartDashboard.putBoolean("Manual Override Press", SmartDashboard.getBoolean("Manual Override Press", false));

    SmartDashboard.putBoolean("Should we blame hardware?", true);    
    //Slider things VARIABLES
    m_outtakeSpeed = SmartDashboard.getNumber("Outtake Speed", m_outtakeSpeed);
    m_intakeSpeed = SmartDashboard.getNumber("Intake Speed", m_intakeSpeed);
    SmartDashboard.putNumber("Outtake Speed", m_outtakeSpeed);
    SmartDashboard.putNumber("Intake Speed", m_intakeSpeed);
    // This method will be called once per scheduler run
    // We will have a pull in fast and slow and a push out fast and slow
    // When we pull in we will use the beam break sensor to stop the motor

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
