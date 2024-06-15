// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// import java.util.function.DoubleSupplier;
// import java.util.function.Supplier;
// import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

/** An example command that uses an example subsystem. */
public class Pivot2d extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final SwerveSubsystem swerveSubsystem;
    private final double targetRotRadians;
    private double yawSpeed;
    private final Translation2d pivotPoint2d;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public Pivot2d(SwerveSubsystem swerveSubsystem, double targetRotRadians, Translation2d pivotPoint2d) {
        this.swerveSubsystem = swerveSubsystem;
        this.targetRotRadians = targetRotRadians;
        this.pivotPoint2d = pivotPoint2d;
        this.yawSpeed = 0;

        addRequirements(swerveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        yawSpeed = swerveSubsystem.StepTowards(swerveSubsystem.getRotation2d().getRadians(), targetRotRadians, AutoConstants.kMaxAngularSpeedRadiansPerSecond);
        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        // Relative to field
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                0, 0, yawSpeed, swerveSubsystem.getRotation2d());
        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds, pivotPoint2d);
        // state.angle.getRadians());
        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (Math.abs(yawSpeed) <= 0.05
                & Math.abs(yawSpeed) <= 0.05) {
            return true;
        } else {
            return false;
        }
    }
}
