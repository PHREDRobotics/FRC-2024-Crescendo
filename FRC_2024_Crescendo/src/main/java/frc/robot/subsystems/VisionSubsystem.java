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
  public boolean m_IsLimeLightCentered = false;
  NetworkTable m_table = NetworkTableInstance.getDefault().getTable("limelight");
  double tv = m_table.getEntry("tv").getDouble(0);
  double tx = m_table.getEntry("tx").getDouble(0);
  double ty = m_table.getEntry("ty").getDouble(0);
  double ta = m_table.getEntry("ta").getDouble(0);
  double tid = m_table.getEntry("tid").getDouble(0);

  public VisionSubsystem() {
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  // public Command VisionCommand() {

  //   // Inline construction of command goes here.
  //   // Subsystem::RunOnce implicitly requires `this` subsystem.
  //   return runOnce(
  //       () -> {
  //         /* one-time action goes here */
  //       });
  // }
  // We will need to move accordingly but I think that will go elsewhere
  public void vision() {
    m_table = NetworkTableInstance.getDefault().getTable("limelight");
    tv = m_table.getEntry("tv").getDouble(0);
    tx = m_table.getEntry("tx").getDouble(0);
    ty = m_table.getEntry("ty").getDouble(0);
    ta = m_table.getEntry("ta").getDouble(0);
    tid = m_table.getEntry("tid").getDouble(0);
    SmartDashboard.putBoolean("Sees target?", !(tid == -1.0));
    SmartDashboard.putNumber("Target:", tid);
    SmartDashboard.putNumber("Limelight a value.", ta);
    SmartDashboard.putNumber("Limelight y value.", ty);
    SmartDashboard.putNumber("Limelight x value.", tx);
    SmartDashboard.putNumber("Limelight v value.", tv);
    SmartDashboard.putBoolean("Is the target in range?", m_LimelightHasValidTarget);
    SmartDashboard.putBoolean("Is the target centered", m_IsLimeLightCentered);
  }
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
     m_table = NetworkTableInstance.getDefault().getTable("limelight");
    tv = m_table.getEntry("tv").getDouble(0);
    tx = m_table.getEntry("tx").getDouble(0);
    ty = m_table.getEntry("ty").getDouble(0);
    ta = m_table.getEntry("ta").getDouble(0);
    tid = m_table.getEntry("tid").getDouble(0);
    SmartDashboard.putBoolean("Sees target?", !(tid == -1.0));
    SmartDashboard.putNumber("Target:", tid);
    SmartDashboard.putNumber("Limelight a value.", ta);
    SmartDashboard.putNumber("Limelight y value.", ty);
    SmartDashboard.putNumber("Limelight x value.", tx);
    SmartDashboard.putNumber("Limelight v value.", tv);
    SmartDashboard.putBoolean("Is the target in range?", m_LimelightHasValidTarget);
    SmartDashboard.putBoolean("Is the target centered", m_IsLimeLightCentered);
    // This method will be called once per scheduler run
    // It should be able to recognize the april tags, which allows us to push a
    // button so it lines itself
    // up and scores in the amp or speaker based on which and where the april tag is

    // SmartDashboard.put variable type ("name", what you want it to display);

    // now get the network table that corresponds to the SmartDashboard class of
    // WPILib
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
