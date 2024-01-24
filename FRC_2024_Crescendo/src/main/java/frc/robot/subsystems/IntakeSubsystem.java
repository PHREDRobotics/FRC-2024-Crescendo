
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TestConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {

  private CANSparkMax LeftMotor = new CANSparkMax(TestConstants.kTestMotorCanId, MotorType.kBrushless);
  private CANSparkMax RightMotor = new CANSparkMax(TestConstants.kTestMotorCanIdTwo, MotorType.kBrushless);

  private SparkLimitSwitch m_forwardLimit = LeftMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);

  public IntakeSubsystem() {
    super();
    
    m_forwardLimit.enableLimitSwitch(false);
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

  public void pickUpNote() {
    // This will stop when the beam in our beam break sensor is broken
    if (m_forwardLimit.isPressed()) {
      m_forwardLimit.enableLimitSwitch(true);
    } else {
      m_forwardLimit.enableLimitSwitch(false);
    }
    LeftMotor.set(speedConvert(1));
    RightMotor.set(-speedConvert(1));
  }

  public void ejectToShooter() {
    // This will be slower than ejectToAmp

  }

  public void ejectToAmp() {
    // This will be faster ten ejectToShooter

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // We will have a pull in fast and slow and a push out fast and slow
    // When we pull in we will use the beam break sensor to stop the motor
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
