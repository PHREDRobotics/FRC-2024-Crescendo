// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class VisionSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public boolean m_LimelightHasValidTarget = false;
  public boolean m_IsLimeLightCentered =  false;

  public VisionSubsystem() {
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command VisionCommand() {
   
    
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  public void readAprilTag() {
    // we will use the network table
      SmartDashboard.putBoolean("Sees target?", m_LimelightHasValidTarget);

    double tv = NetworkTableInstance.getDefault().getTable("AprilTags_pipeline").getEntry("tv").getDouble(0);
    double tx = NetworkTableInstance.getDefault().getTable("AprilTags_pipeline").getEntry("tx").getDouble(0);
    double ty = NetworkTableInstance.getDefault().getTable("AprilTags_pipeline").getEntry("ty").getDouble(0);
    double ta = NetworkTableInstance.getDefault().getTable("AprilTags_pipeline").getEntry("ta").getDouble(0);
      // this should return which april tag we are near


       if (tv > 2.0)
    {
      m_LimelightHasValidTarget = false;

    }else {

    m_LimelightHasValidTarget = true;
  }

  //this is telling it to move to the center if the april tag is not centered
         if (ty > 0.5)
    {
      m_IsLimeLightCentered = false;

    }else {

    m_IsLimeLightCentered = true;
  }
    }
  // We will need to move accordingly but I think that will go elsewhere

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // It should be able to recognize the april tags, which allows us to push a
    // button so it lines itself
    // up and scores in the amp or speaker based on which and where the april tag is
    SmartDashboard.putBoolean("Is the target in range?", m_LimelightHasValidTarget);
    SmartDashboard.putBoolean("Is the target centered", m_IsLimeLightCentered);
   // SmartDashboard.put variable type ("name", what you want it to display);

   


// now get the network table that corresponds to the SmartDashboard class of WPILib
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
