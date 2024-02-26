// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkLowLevel.MotorType;
import java.util.function.DoubleSupplier;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

/**
 * Subsystem for the robot's arm
 */
public class ArmSubsystem extends SubsystemBase {

    double kP;
    double kI;
    double kD = 0;
    private double kS;
    private double kG;
    private double kV;
    private double kA;
    private double kDt;
    private int m_can_id;
    private MotorType m_motor_type;
    private DigitalInput m_limit_switch;
    private RelativeEncoder armEncoder;
    private CANSparkMax armMotor;
    private double max_velocity;
    private double max_acceleration;
    private double voltage;
    private PIDController pidController;
    ArmFeedforward feedforward;
    private TrapezoidProfile profile;
    private TrapezoidProfile.State goal;
    private TrapezoidProfile.State setpoint;

    public String armPosition;

    /**
     * Subsystem for the robot's arm
     * 
     * @param canID     ID of the arm's spark max controller
     * @param motorType Type of motor the arm is
     */

    public ArmSubsystem() {
        m_can_id = Constants.ArmConstants.kArmControllerPort;
        m_motor_type = CANSparkMax.MotorType.kBrushless;
        m_limit_switch = new DigitalInput(Constants.ArmConstants.kLimitSwitchControllerPort);

        kP = .6;
        kI = 0;
        kD = 0;

        // kP = SmartDashboard.getNumber("kP", kP);
        // kI = SmartDashboard.getNumber("kI", kI);
        // kD = SmartDashboard.getNumber("kD", kD);

        kDt = 0.2;

        kS = 0;
        kG = .1;
        kV = 0;
        kA = 0;

        max_velocity = 1;
        max_acceleration = .5;

        // max_velocity = SmartDashboard.getNumber("Max Vel", max_velocity);
        // max_acceleration = SmartDashboard.getNumber("Max Accel",
        // max_acceleration);

        armMotor = new CANSparkMax(m_can_id, m_motor_type);
        armEncoder = armMotor.getEncoder();
        feedforward = new ArmFeedforward(kS, kG, kV, kA);
        setpoint = new TrapezoidProfile.State();
        pidController = new PIDController(kP, kI, kD);
        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(max_velocity, max_acceleration));
        goal = new TrapezoidProfile.State();
        setpoint = new TrapezoidProfile.State();

        armMotor.setIdleMode(IdleMode.kBrake);
    }

    /**
     * Move the arm to a desired position
     * 
     * @param position Target position in encoder ticks
     */
    public void moveToPosition(int position) {
        switch(position) {
            case Constants.ArmConstants.kArmPickup:
                armPosition = "Pickup";
                break;
            case Constants.ArmConstants.kArmAmp:
                armPosition = "Amp";
                break;
            case Constants.ArmConstants.kArmUp:
                armPosition = "Up";
                break;
            case Constants.ArmConstants.kArmShooter:
                armPosition = "Shooter";
                break;
        }

        goal = new TrapezoidProfile.State(position, 0);
        setpoint = profile.calculate(kDt, setpoint, goal);
        armMotor.setVoltage(pidController.calculate(armEncoder.getPosition(), setpoint.position)
                + feedforward.calculate(Math.toRadians(armEncoder.getPosition()), setpoint.velocity));
    }

    /**
     * Send voltage directly to the motor
     * 
     * @param power in volts
     */
    public void setRawPower(double power) {
        voltage = power * Constants.ArmConstants.kVoltageMultiplier;
        armMotor.setVoltage(voltage);
    }

    public void setSpeed(DoubleSupplier speed) {
        double vroom = speed.getAsDouble();
        armMotor.set(vroom);
    }

    public boolean limitSwitchTriggered() {
        return m_limit_switch.get();
    }

    /**
     * Reset the encoder on the arm
     */
    public void resetEncoders() {
        armMotor.set(0);
        armEncoder.setPosition(0);
    }

    @Override
    public void simulationPeriodic() {
        SmartDashboard.putNumber("position", armEncoder.getPosition());
        // SmartDashboard.putBoolean("Limit Switch:", m_limit_switch.get());
        SmartDashboard.putNumber("voltage: ", voltage);

        SmartDashboard.putNumber("kP", kP);
        SmartDashboard.putNumber("kI", kI);
        SmartDashboard.putNumber("kD", kD);
        SmartDashboard.putNumber("Setpoint", setpoint.position);

        SmartDashboard.putNumber("Max Vel", max_velocity);
        SmartDashboard.putNumber("Max Accel", max_acceleration);
        
        
        SmartDashboard.putString("Gameboard/Arm Position", armPosition);
    }

    public void periodic() {
        SmartDashboard.putNumber("position", armEncoder.getPosition());
        SmartDashboard.putBoolean("Limit Switch:", this.limitSwitchTriggered());
        kDt = kDt + 1 / 50;
    }
}