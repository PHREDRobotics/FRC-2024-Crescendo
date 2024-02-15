package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.GrabberConstants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj.Timer;
import javax.print.attribute.standard.RequestingUserName;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {
  private static final Timer m_timer = new Timer();

  public CANSparkMax m_upMotor = new CANSparkMax(ShooterConstants.kLeftShooterControllerPort, MotorType.kBrushless);
  public CANSparkMax m_downMotor = new CANSparkMax(ShooterConstants.kRightShooterControllerPort, MotorType.kBrushless);
  
 // public SparkLimitSwitch m_forwardLimit = m_upMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

  public double m_ShooterSpeed = ShooterConstants.kShooterSpeed;

  // public SparkLimitSwitch m_forwardLimit =
  // m_upMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);

  // public ShooterSubsystem() {
  //   super();
  //   // m_forwardLimit.enableLimitSwitch(false);
  // }

 

  public void Shoot() {
    m_timer.reset();
    m_upMotor.set(speedConvert(m_ShooterSpeed));
    m_downMotor.set(speedConvert(m_ShooterSpeed));
    m_timer.start();
  }


  public double speedConvert(double inSpeed) {

    if (inSpeed < 0.2 && inSpeed > -0.2) {
      return 0.0;
    }

    return inSpeed;
  }

  public void stopShooter() {
    m_upMotor.set(0);
    m_downMotor.set(0);
  }  

  public static boolean shooterIsTimeDone() {
    return m_timer.hasElapsed(Constants.ShooterConstants.kShooterTime);

  }

  public void ejectToShooter() {
    // This will be slower than ejectToAmp

  }

  public void ejectToAmp() {
    
    // This will be faster ten ejectToShooter
  }

  @Override
  public void periodic() {

     
    //Slider things VARIABLES
    m_ShooterSpeed = SmartDashboard.getNumber("Shooter Speed", m_ShooterSpeed);
    SmartDashboard.putNumber("Shooter Speed", m_ShooterSpeed);
    // This method will be called once per scheduler run
    // We will have a pull in fast and slow and a push out fast and slow
    // When we pull in we will use the beam break sensor to stop the motor

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
