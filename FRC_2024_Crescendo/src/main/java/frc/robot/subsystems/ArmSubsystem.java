// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TestConstants;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

/**
 * Arm Subsystem for PHRED's really cool robot
 */
public class ArmSubsystem extends SubsystemBase {

  private int m_can_id;
  private MotorType m_motor_type;

  private CANSparkMax armMotor = new CANSparkMax(m_can_id, m_motor_type);
  SparkPIDController pidController = armMotor.getPIDController();
  RelativeEncoder armEncoder = armMotor.getEncoder();

  public ArmSubsystem(int canID, MotorType motorType) {
    pidController.setFeedbackDevice(armEncoder);

    pidController.setP(0.01);
    pidController.setI(0.001);
    pidController.setD(0.001);

    m_can_id = canID;
    m_motor_type = motorType;

  }

  /**
   * Move the arm to a desired position
   * 
   * @param position 5 = low, 20 = mid, 30 = high
   */
  public void moveToPosition(double position) {
    pidController.setReference(position, CANSparkMax.ControlType.kPosition);
  }

  public void moveToFLoor() {

  }

  public void moveToMiddle() {

  }

  public void MoveToTop() {

  }

  public void grab() {

  }

  public void prepareToLoad() {

  }

  public void loadAmp() {

  }

  public void resetEncoders() {
    // this will move it up until a sensor sees that its at the top and then it
    // stops and resets the encoders
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // It will be able to grab a note and either feed it to the shooter or score on
    // the amp

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
