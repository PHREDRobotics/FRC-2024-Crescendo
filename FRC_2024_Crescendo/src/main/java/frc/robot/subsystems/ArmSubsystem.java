package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

/** A robot arm subsystem that moves with a motion profile. */
public class ArmSubsystem extends PIDSubsystem {
    private final DigitalInput m_limit_switch;

    private final CANSparkMax m_motor = new CANSparkMax(Constants.ArmConstants.kArmControllerPort,
            CANSparkMax.MotorType.kBrushless);
    private final ArmFeedforward m_feedforward = new ArmFeedforward(
            Constants.ArmConstants.kSVolts, Constants.ArmConstants.kGVolts,
            Constants.ArmConstants.kVVoltSecondPerRad, Constants.ArmConstants.kAVoltSecondSquaredPerRad);
    PIDController m_pidController = new PIDController(Constants.ArmConstants.kP, Constants.ArmConstants.kI, Constants.ArmConstants.kD);

    /** Create a new ArmSubsystem. */
    public ArmSubsystem() {
        super(new PIDController(Constants.ArmConstants.kP, Constants.ArmConstants.kI, Constants.ArmConstants.kD));
        m_limit_switch = new DigitalInput(Constants.ArmConstants.kLimitSwitchControllerPort);
        disable();
    }

    public void resetEncoders() {
        m_motor.set(0);
        m_motor.getEncoder().setPosition(0);
        moveToPosition(ArmConstants.kArmOffsetRads);
    }

    public boolean limitSwitchTriggered() {
        return m_limit_switch.get();
    }

    public double getArmRadians() {
        return -Constants.k2pi * (m_motor.getEncoder().getPosition() / 40) + ArmConstants.kArmOffsetRads;
    }

    @Override
    public void periodic() {
        super.periodic();

        // if (limitSwitchTriggered()) {
        //     resetEncoders();

        // }

        SmartDashboard.putBoolean("Gameboard/Limit Switch:", this.limitSwitchTriggered());
        SmartDashboard.putNumber("Gameboard/Arm Position:", getArmRadians());
        SmartDashboard.putNumber("Gameboard/Arm Position in Encoder Rotations:", m_motor.getEncoder().getPosition());
    }

    @Override
    public void useOutput(double output, double setpoint) {
        SmartDashboard.putNumber("Gameboard/Arm Target:", setpoint);
        
        // Add the feedforward to the PID output to get the motor output
        double volts = -m_pidController.calculate(getArmRadians(), setpoint);
        SmartDashboard.putNumber("Gameboard/Arm Volts:", volts);
                        
        m_motor.setVoltage(volts);
    }

    public void setRawPower(double power) {
        power = power * Constants.ArmConstants.kVoltageMultiplier;
        m_motor.setVoltage(power);
    }

    @Override
    protected double getMeasurement() {
        return getArmRadians();
    }

    /**
     * 
     * @param position In radians
     */
    public void moveToPosition(double position) {
        m_controller.setSetpoint(position);
    }
}