// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GrabberConstants;
//import frc.robot.commands.DriveMotor;
//import frc.robot.commands.OuttakeCommand;
//import frc.robot.subsystems.MotorTestSubsystem;
import frc.robot.commands.*;
import frc.robot.composition.ShootTwoNotes;
import frc.robot.controls.FlightStick;
import frc.robot.controls.LogitechPro;
import frc.robot.subsystems.*;

import java.util.List;

// import frc.robot.subsystems.MotorTestSubsystem;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
// import frc.robot.commands.Autos;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
        SendableChooser<Command> m_chooser = new SendableChooser<>();

        // The robot's subsystems and commands are defined here...
        // private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
        // private final MotorTestSubsystem motorTestSubsystem = new
        // MotorTestSubsystem();
        // Define Joysticks
        private final LogitechPro joyStick = new LogitechPro(1);
        // Replace with CommandPS4Controller or CommandJoystick if needed
        private final XboxController driverJoystick = new XboxController(0);

        // Define subsystems
        private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
        private final VisionSubsystem visionSubsystem = new VisionSubsystem();
        private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(joyStick);
        private final LiftSubsystem liftSubsystem = new LiftSubsystem();
        private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
        private final ArmSubsystem armSubsystem = new ArmSubsystem();

        

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        // 4. Construct command to follow trajectory
       /*  SwerveControllerCommand TrajectoryShootNote = new SwerveControllerCommand(
                        swerveSubsystem.getTrajectory(
                                        new Pose2d(0.0, 0.0, new Rotation2d()),
                                        new Translation2d(2.5, 0.0),
                                        new Pose2d(5.0, 0.0, new Rotation2d())),
                        swerveSubsystem::getPose,
                        DriveConstants.kDriveKinematics,
                        swerveSubsystem.xController,
                        swerveSubsystem.yController,
                        swerveSubsystem.thetaController,
                        swerveSubsystem::setModuleStates,
                        swerveSubsystem);*/
                        

        // SwerveControllerCommand TrajectoryNoteShoot = new SwerveControllerCommand(
        //                 swerveSubsystem.getTrajectory(
        //                                 swerveSubsystem.getPose(),
        //                                 new Translation2d(0.0, 0.0),
        //                                 new Pose2d(0.0, 0.0, new Rotation2d())),
        //                 swerveSubsystem::getPose,
        //                 DriveConstants.kDriveKinematics,
        //                 swerveSubsystem.xController,
        //                 swerveSubsystem.yController,
        //                 swerveSubsystem.thetaController,
        //                 swerveSubsystem::setModuleStates,
        //                 swerveSubsystem);

        // // 5. Add some init and wrap-up, and return everything
        // final Command ShootTwoNotes = new SequentialCommandGroup(new AutoResetArmEncoder(armSubsystem),
        //                 new ParallelCommandGroup(
        //                                 new ShooterCommand(shooterSubsystem),
        //                                 new OuttakeCommand(intakeSubsystem)),


        //                 new ParallelDeadlineGroup(new IntakeCommand(intakeSubsystem),
        //                                 //TrajectoryShootNote,
        //                                 new ArmMotor(Constants.ArmConstants.kArmPickup, armSubsystem)),
        //                                 new InstantCommand(() -> swerveSubsystem.stopModules()),
        //                                 TrajectoryNoteShoot,
        //                                 new InstantCommand(() -> swerveSubsystem.stopModules()),
        //                                 new ParallelCommandGroup(

        //                                 new ShooterCommand(shooterSubsystem),
        //                                 new OuttakeCommand(intakeSubsystem)));

        public RobotContainer() {
                m_chooser.setDefaultOption("Gameboard/Shoot Two Notes", new ShootTwoNotes(armSubsystem, intakeSubsystem,shooterSubsystem,swerveSubsystem));
                m_chooser.addOption("Gameboard/Drive Foward", new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.zeroHeading()),
                new GoToPose2d(swerveSubsystem, new Translation2d(3.0,0.0))));

                // m_chooser.addOption("Complex Auto", ShootNote);

                SmartDashboard.putData(m_chooser);
                configureBindings();
        }

        /**
         * Use this method to define your trigger->command mappings. Triggers can be
         * created via the
         * {@link Trigger trigger(java.util.function.BooleanSupplier)} constructor with
         * an arbitrary
         * predicate, or via the named factories in {@link
         * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
         * {@link
         * CommandXboxController
         * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
         * PS4} controllers or
         * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
         * joysticks}.
         */

        private void configureBindings() {
                // Define triggers
                Trigger xButton = new JoystickButton(driverJoystick, Constants.OIConstants.kXButton);
                Trigger yButton = new JoystickButton(driverJoystick, Constants.OIConstants.kYButton);
                Trigger aButton = new JoystickButton(driverJoystick, Constants.OIConstants.kAButton);
                Trigger bButton = new JoystickButton(driverJoystick, Constants.OIConstants.kBButton);
                Trigger leftBumper = new JoystickButton(driverJoystick, Constants.OIConstants.kLeftBumper);
                Trigger rightBumper = new JoystickButton(driverJoystick, Constants.OIConstants.kRightBumper);
                Trigger startButton = new JoystickButton(driverJoystick, Constants.OIConstants.kStartButton);
                Trigger maryButton = new JoystickButton(joyStick, 2);

                // Set default commands
                // visionSubsystem.setDefaultCommand(new VisionCommand(visionSubsystem));

                armSubsystem.setDefaultCommand(new ManualArmCmd(() -> (driverJoystick.getLeftY()), armSubsystem));

                Trigger dPadUp = new POVButton(driverJoystick, 0);
                Trigger dPadDown = new POVButton(driverJoystick, 180);

                // Configure Bindings
                
                swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                                swerveSubsystem,
                                () -> joyStick.getBackToFront(),
                                () -> joyStick.getLeftToRight(),
                                () -> joyStick.getYaw(),
                                () -> swerveSubsystem.throttleAdjust(joyStick.getThrottle()),
                                () -> joyStick.getTrigger()));
                
                /*
                liftSubsystem.setDefaultCommand(new ManualUnretractLift(
                                () -> leftBumper.getAsBoolean(),
                                () -> rightBumper.getAsBoolean(),
                                liftSubsystem));
                */

                liftSubsystem.setDefaultCommand(new ManualLiftCmd(
                        () -> driverJoystick.getLeftTriggerAxis(),
                        () -> driverJoystick.getRightTriggerAxis(),
                        liftSubsystem));

                // Configure gamepad buttons
                // xButton.onTrue(new ArmMotor(Constants.ArmConstants.kArmLow, armSubsystem));
                // yButton.onTrue(new ArmMotor(Constants.ArmConstants.kArmMid, armSubsystem));
                // bButton.onTrue(new ArmMotor(Constants.ArmConstants.kArmHigh, armSubsystem));
                // yButton.onTrue(new UnretractLift(liftSubsystem));
                //yButton.whileTrue(new ManualLiftCmd(
                //                () -> driverJoystick.getLeftTriggerAxis(),
                //                () -> driverJoystick.getRightTriggerAxis(),
                //                liftSubsystem));
                //xButton.onTrue(new OuttakeCommand(intakeSubsystem));
                //bButton.onTrue(new ParallelCommandGroup(new ShooterCommand(shooterSubsystem),
                //                new OuttakeCommand(intakeSubsystem)));
                //aButton.onTrue(new IntakeCommand(intakeSubsystem));
                // leftBumper.onTrue(new AutoResetArmEncoder(armSubsystem));
                maryButton.onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
                leftBumper.whileTrue(new ManualUnretractLift(() -> true, () -> false, liftSubsystem));
                rightBumper.whileTrue(new ManualUnretractLift(() -> false, () -> true, liftSubsystem));

                aButton.onTrue(new SequentialCommandGroup(new IntakeCommand(intakeSubsystem), new AutoResetArmEncoder(armSubsystem)));
                xButton.onTrue(new OuttakeCommand(intakeSubsystem));
                bButton.onTrue(new ParallelCommandGroup(new ShooterCommand(shooterSubsystem),
                                new OuttakeCommand(intakeSubsystem)));

                startButton.onTrue(new AutoResetArmEncoder(armSubsystem));
                
                dPadUp.onTrue(armSubsystem.setArmGoalCommand(Constants.ArmConstants.kArmShooter));
                dPadDown.onTrue(armSubsystem.setArmGoalCommand(Constants.ArmConstants.kArmPickup));
                yButton.onTrue(armSubsystem.setArmGoalCommand(Constants.ArmConstants.kArmAmp));

                visionSubsystem.setDefaultCommand(new VisionCommand(visionSubsystem));
                
                /*
                 * Use this to pass the autonomous command to the main {@link Robot} class.
                 *
                 * @return the command to run in autonomous
                 */

                // return null;
                // aButton.onTrue(new ArmMotor(Constants.ArmConstants.kArmPickup, armSubsystem));
                // xButton.onTrue(new ArmMotor(Constants.ArmConstants.kArmAmp, armSubsystem));
                

                //Please don't break this
                //yButton.onTrue(new ArmMotor(Constants.ArmConstants.kArmAmp, armSubsystem));
                //bButton.onTrue(new ParallelCommandGroup(new ShooterCommand(shooterSubsystem),
                //               new OuttakeCommand(intakeSubsystem)));
                //xButton.onTrue(new OuttakeCommand(intakeSubsystem));
                //aButton.onFalse(new IntakeCommand(intakeSubsystem));
        }

        /*
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        // public Command getAutonomousCommand() {

        // return null;
        // }
        public Command getAutonomousCommand() {
                // The selected command will be run in autonomous
                return m_chooser.getSelected();
        }
        public Command getTeleopCommand() {
                return new AutoResetArmEncoder(armSubsystem);
        }
}