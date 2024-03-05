package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

/** A robot arm subsystem that moves with a motion profile. */
public class ArmSubsystem extends TrapezoidProfileSubsystem {
    private final DigitalInput m_limit_switch;

    private final CANSparkMax m_motor = new CANSparkMax(Constants.ArmConstants.kArmControllerPort,
            CANSparkMax.MotorType.kBrushless);
    private final ArmFeedforward m_feedforward = new ArmFeedforward(
            Constants.ArmConstants.kSVolts, Constants.ArmConstants.kGVolts,
            Constants.ArmConstants.kVVoltSecondPerRad, Constants.ArmConstants.kAVoltSecondSquaredPerRad);
    PIDController m_pidController = new PIDController(Constants.ArmConstants.kP, 0.0, 0.0);

    /** Create a new ArmSubsystem. */
    public ArmSubsystem() {
        super(
                new TrapezoidProfile.Constraints(
                        Constants.ArmConstants.kMaxVelocityRadPerSecond,
                        Constants.ArmConstants.kMaxAccelerationRadPerSecSquared),
                Constants.ArmConstants.kArmOffsetRads);
        m_limit_switch = new DigitalInput(Constants.ArmConstants.kLimitSwitchControllerPort);

    }

    public void resetEncoders() {
        m_motor.set(0);
        m_motor.getEncoder().setPosition(0);
    }

    public boolean limitSwitchTriggered() {
        return m_limit_switch.get();
    }

    public double getEncoderRadians(){
        return Constants.k2pi * (m_motor.getEncoder().getPosition() / 40) + ArmConstants.kArmOffsetRads;
    }


    public void periodic() {
        if (limitSwitchTriggered()) {
            resetEncoders();
            
        }
       
        SmartDashboard.putBoolean("Gameboard/Limit Switch:", this.limitSwitchTriggered());
        SmartDashboard.putNumber("Gameboard/Arm Position:", getEncoderRadians());
                SmartDashboard.putNumber("Gameboard/Arm Position in Encoder Ticks:", m_motor.getEncoder().getPosition());

            }

    @Override
    public void useState(TrapezoidProfile.State setpoint) {
        // Calculate the feedforward from the sepoint
        double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
        // Add the feedforward to the PID output to get the motor output

        m_motor.setVoltage(m_pidController.calculate(getEncoderRadians(), setpoint.position)
                + feedforward);
    }

    public void setRawPower(double power){
        power = power * Constants.ArmConstants.kVoltageMultiplier;
        m_motor.setVoltage(power);
    }

    public Command setArmGoalCommand(double kArmOffsetRads) {
        return Commands.runOnce(() -> setGoal(kArmOffsetRads), this);
    }
}