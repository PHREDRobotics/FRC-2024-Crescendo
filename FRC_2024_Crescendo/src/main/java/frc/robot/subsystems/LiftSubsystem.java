package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LiftSubsystem extends SubsystemBase {

  private CANSparkMax leftLiftMotor;
  private CANSparkMax rightLiftMotor;
  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;
  private AHRS navXMicro;

  /**
   * Subsystem to control the lift
   */
  public LiftSubsystem() {
    leftLiftMotor = new CANSparkMax(Constants.LiftConstants.kLeftLiftControllerPort, MotorType.kBrushless);
    rightLiftMotor = new CANSparkMax(Constants.LiftConstants.kRightLiftControllerPort, MotorType.kBrushless);
    leftEncoder = leftLiftMotor.getEncoder();
    rightEncoder = rightLiftMotor.getEncoder();
    try {
      navXMicro = new AHRS(SPI.Port.kMXP);
    } catch (RuntimeException ex) {
      DriverStation.reportError(ex.getMessage(), true);
    }
    Timer.delay(1.0);

    addChild("NavX Micro", navXMicro);
  }

  /**
   * Reset Lift Motor Encoders
   */
  public void resetEncoders() {
    leftLiftMotor.set(0);
    rightLiftMotor.set(0);

    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  /**
   * Automatically lifts the robot, must be called in a loop
   * 
   * @param speed Value between 0 and 1
   */
  public void AutoLift(double speed) {
    double roll = navXMicro.getRoll();
    boolean checkLevel = false;
    leftLiftMotor.setIdleMode(IdleMode.kBrake);
    rightLiftMotor.setIdleMode(IdleMode.kBrake);
    if (roll > 1) {
      leftLiftMotor.set(0);
      checkLevel = true;
    } else if (roll < -1) {
      rightLiftMotor.set(0);
      checkLevel = true;
    } else if (checkLevel && roll < 1 && roll > -1) {
      leftLiftMotor.set(0);
      rightLiftMotor.set(0);
    }
  }

  public double getLeftEncoder() {
    return leftEncoder.getPosition();
  }

  public void extendLeftLift() {
    leftLiftMotor.set(.1);
  }

  public boolean isLeftDone() {
    if (leftEncoder.getPosition() == 0) {

      return true;

    } else {

      return false;
    }
  }

  public void extendRightLift() {
    leftLiftMotor.set(.1);
  }

  public void stopLeftLift() {
    leftLiftMotor.set(0);
  }

  public boolean isRightDone() {
    if (rightEncoder.getPosition() == 0) {

      return true;

    } else {

      return false;
    }
  }

  public void stopRightLift() {
    rightLiftMotor.set(0);
  }

  public double getRightEncoder() {
    return rightEncoder.getPosition();
  }

  /**
   * Set the power of the left lift motor
   * 
   * @param left_power Value between 0 and 1
   */
  public void setRawLeftPower(DoubleSupplier left_power) {
    leftLiftMotor.set(left_power.getAsDouble());
  }

  /**
   * Set the power of the right lift motor
   * 
   * @param right_power Value between 0 and 1
   */
  public void setRawRightPower(DoubleSupplier right_power) {
    rightLiftMotor.set(right_power.getAsDouble());
  }

  public void changeLiftMode(IdleMode mode) {
    leftLiftMotor.setIdleMode(mode);
    rightLiftMotor.setIdleMode(mode);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Gameboard/Left Power", leftLiftMotor.get());
    SmartDashboard.putNumber("Gameboard/Right Power", rightLiftMotor.get());
    SmartDashboard.putNumber("Left Encoder", leftEncoder.getPosition());
    SmartDashboard.putNumber("Right Encoder", rightEncoder.getPosition());
    SmartDashboard.putNumber("NavX y", navXMicro.getPitch());
    SmartDashboard.putNumber("NavX x", navXMicro.getRoll());
    SmartDashboard.putNumber("NavX z", navXMicro.getYaw());

  }
}