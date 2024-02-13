// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

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
    private RelativeEncoder armEncoder;
    private CANSparkMax armMotor;
    private PIDController pidController;
    ArmFeedforward feedforward;
    private final TrapezoidProfile profile;
    private TrapezoidProfile.State goal;
    private TrapezoidProfile.State setpoint;
    /**
     * Subsystem for the robot's arm
     * @param canID ID of the arm's spark max controller
     * @param motorType Type of motor the arm is
     */
    public ArmSubsystem(
        int canID, MotorType motorType, 
        double p, double i, double d, double dt,
        double s, double g, double v, double a,
        double maxVelocity, double maxAcceleration
        ) {
        this.m_can_id = canID;
        this.m_motor_type = motorType;
        
        this.kP = p;
        this.kI = i;
        this.kD = d;

        this.kDt = dt;

        this.kS = s;
        this.kG = g;
        this.kV = v;
        this.kA = a;

        this.armMotor = new CANSparkMax(m_can_id, m_motor_type);
        this.armEncoder = armMotor.getEncoder();
        this.feedforward = new ArmFeedforward(kS, kG, kV, kA);
        this.profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
        this.goal = new TrapezoidProfile.State();
        this.setpoint = new TrapezoidProfile.State();
        this.pidController = new PIDController(kP, kI, kD);
    }

    /**
     * Move the arm to a desired position
     * @param position Target position in encoder ticks
     */
    public void moveToPosition(double position) {
        this.setpoint = this.profile.calculate(this.kDt, this.setpoint, this.goal);
        this.pidController.setSetpoint(setpoint.velocity);
        this.goal = new TrapezoidProfile.State(position, 0);
        this.armMotor.setVoltage(this.pidController.calculate(this.armEncoder.getPosition(), 0) + this.setpoint.velocity);
    }

    public void setRawPower(DoubleSupplier power) {
        this.armMotor.set(power.getAsDouble());
    }

    /**
     * Reset the encoder on the arm
     */
    public void resetEncoders() {
        this.armMotor.set(0);
        this.armEncoder.setPosition(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("position", this.armEncoder.getPosition());
    }
}
