// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.composition;

import frc.robot.subsystems.*;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

/** An example command that uses an example subsystem. */
public class ShootTwoNotes extends SequentialCommandGroup {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final ArmSubsystem m_ArmSubsystem;
    private final IntakeSubsystem m_IntakeSubsystem;
    private final ShooterSubsystem m_ShoooterSubsystem;
    private final SwerveSubsystem m_SwerveSubsystem;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ShootTwoNotes(ArmSubsystem arm, IntakeSubsystem intake, ShooterSubsystem shoooter, SwerveSubsystem swerve) {
        m_ArmSubsystem = arm;
        m_IntakeSubsystem = intake;
        m_ShoooterSubsystem = shoooter;
        m_SwerveSubsystem = swerve;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(arm);
        addRequirements(intake);
        addRequirements(shoooter);
        addRequirements(swerve);

        SwerveControllerCommand TrajectoryShootNote = new SwerveControllerCommand(
                m_SwerveSubsystem.getTrajectory(
                        new Pose2d(0.0, 0.0, new Rotation2d()),
                        new Translation2d(2.5, 0.0),
                        new Pose2d(5.0, 0.0, new Rotation2d())),
                m_SwerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                m_SwerveSubsystem.xController,
                m_SwerveSubsystem.yController,
                m_SwerveSubsystem.thetaController,
                m_SwerveSubsystem::setModuleStates,
                m_SwerveSubsystem);

        addCommands(
            new InstantCommand(() -> m_SwerveSubsystem.zeroHeading()),
                new AutoResetArmEncoder(m_ArmSubsystem),

                new ParallelCommandGroup(
                        new ShooterCommand(m_ShoooterSubsystem),
                        new OuttakeCommand(m_IntakeSubsystem)),

                new ParallelDeadlineGroup(
                        new IntakeCommand(m_IntakeSubsystem),
                        new SwerveControllerCommand(
                                m_SwerveSubsystem.getTrajectory(
                                        new Pose2d(0.0, 0.0, new Rotation2d()),
                                        new Translation2d(2.5, 0.0),
                                        new Pose2d(5.0, 0.0, new Rotation2d())),
                                m_SwerveSubsystem::getPose,
                                DriveConstants.kDriveKinematics,
                                m_SwerveSubsystem.xController,
                                m_SwerveSubsystem.yController,
                                m_SwerveSubsystem.thetaController,
                                m_SwerveSubsystem::setModuleStates,
                                m_SwerveSubsystem),
                        new ArmMotor(Constants.ArmConstants.kArmPickup, m_ArmSubsystem)),

                new InstantCommand(() -> m_SwerveSubsystem.stopModules()),

                new ParallelCommandGroup(
                    
                        // new SwerveControllerCommand(
                        //         m_SwerveSubsystem.getTrajectory(
                        //                 new Pose2d(1.0, 0.0, new Rotation2d()),
                        //                 new Translation2d(0.5, 0.0),
                        //                 new Pose2d(0.0, 0.0, new Rotation2d())),
                        //         m_SwerveSubsystem::getPose,
                        //         DriveConstants.kDriveKinematics,
                        //         m_SwerveSubsystem.xController,
                        //         m_SwerveSubsystem.yController,
                        //         m_SwerveSubsystem.thetaController,
                        //         m_SwerveSubsystem::setModuleStates,
                        //         m_SwerveSubsystem),
                        new ArmMotor(Constants.ArmConstants.kArmShooter, m_ArmSubsystem)),

                new InstantCommand(() -> m_SwerveSubsystem.stopModules()),

                new ParallelCommandGroup(
                        new ShooterCommand(m_ShoooterSubsystem),
                        new OuttakeCommand(m_IntakeSubsystem)));
    }

    // // Called when the command is initially scheduled.
    // @Override
    // public void initialize() {}

    // // Called every time the scheduler runs while the command is scheduled.
    // @Override
    // public void execute() {}

    // // Called once the command ends or is interrupted.
    // @Override
    // public void end(boolean interrupted) {}

    // // Returns true when the command should end.
    // @Override
    // public boolean isFinished() {
    // return false;
    // }
}
