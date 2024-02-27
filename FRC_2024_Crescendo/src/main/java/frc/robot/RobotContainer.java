// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;
import frc.robot.composition.ShootTwoNotes;
import frc.robot.controls.LogitechPro;
import frc.robot.subsystems.*;

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

        private final LogitechPro joyStick = new LogitechPro(1);

        private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
        private final VisionSubsystem visionSubsystem = new VisionSubsystem();
        private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(joyStick);
        private final LiftSubsystem liftSubsystem = new LiftSubsystem();
        private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
        private final ArmSubsystem armSubsystem = new ArmSubsystem();


        /*
         * DigitalInput limitSwitch = new
         * DigitalInput(Constants.ArmConstants.kLimitSwitchControllerPort);
         * private final ArmSubsystem armSubsystem = new ArmSubsystem(
         * Constants.ArmConstants.kArmControllerPort, CANSparkMax.MotorType.kBrushless,
         * limitSwitch,
         * 0.6, 0, 0, 0.02,
         * 0, 0.1, 0, 0,
         * 10, 5);
         */
        // Replace with CommandPS4Controller or CommandJoystick if needed
        private final XboxController driverJoystick = new XboxController(0);


        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */

        public RobotContainer() {
                m_chooser.setDefaultOption("Shoot Two Notes",
                                new ShootTwoNotes(armSubsystem, intakeSubsystem, shooterSubsystem, swerveSubsystem));

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
                Trigger dpadUp = new POVButton(driverJoystick, 0);
                Trigger dpadDown = new POVButton(driverJoystick, 180);

                // Set default commands
                // visionSubsystem.setDefaultCommand(new VisionCommand(visionSubsystem));

                swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                                swerveSubsystem,
                                () -> joyStick.getBackToFront(),
                                () -> joyStick.getLeftToRight(),
                                () -> joyStick.getYaw(),
                                () -> swerveSubsystem.throttleAdjust(joyStick.getThrottle()),
                                () -> joyStick.getTrigger()));
                liftSubsystem.setDefaultCommand(new ManualUnretractLift(
                                () -> leftBumper.getAsBoolean(),
                                () -> rightBumper.getAsBoolean(),
                                liftSubsystem));

                // Configure mechanical triggers

                // Please don't break this (correct button mapings).
                yButton.onTrue(new ArmMotor(Constants.ArmConstants.kArmAmp, armSubsystem));
                bButton.onTrue(new ParallelCommandGroup(new ShooterCommand(shooterSubsystem),
                                new OuttakeCommand(intakeSubsystem)));
                xButton.onTrue(new OuttakeCommand(intakeSubsystem));
                aButton.onTrue(new IntakeCommand(intakeSubsystem));
                dpadUp.onTrue(new ArmMotor(Constants.ArmConstants.kArmShooter, armSubsystem));
                dpadDown.onTrue(new ArmMotor(Constants.ArmConstants.kArmPickup, armSubsystem));
                armSubsystem.setDefaultCommand(new ManualArmCmd(() -> (driverJoystick.getLeftY()), armSubsystem));
                maryButton.onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));

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
}