// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public IntakeSubsystem() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    //We will use a beam break sensor
    /*public boolean isBeamBroken(){

    } */
    return false;
  }
public void pickUpNote(){
//This will stop when the beam in our beam break sensor is broken

}

public void ejectToShooter(){
//This will be slower than ejectToAmp
  
}

public void ejectToAmp(){
//This will be faster ten ejectToShooter

}



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //We will have a pull in fast and slow and a push out fast and slow
    //When we pull in we will use the beam break sensor to stop the motor
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
