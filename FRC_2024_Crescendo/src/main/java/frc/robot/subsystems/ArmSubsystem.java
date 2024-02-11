package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

/**
 * Subsystem for the robot's arm
 */
public class ArmSubsystem extends SubsystemBase {

    private int m_can_id;
    private MotorType m_motor_type;
    private RelativeEncoder armEncoder;
    private CANSparkMax armMotor;
    private PIDController pidController;

    /**
     * Subsystem for the robot's arm
     * @param canID ID of the arm's spark max controller
     * @param motorType Type of motor the arm is
     */
    public ArmSubsystem(int canID, MotorType motorType) {
        m_can_id = canID;
        m_motor_type = motorType;

        armMotor = new CANSparkMax(m_can_id, m_motor_type);
        armEncoder = armMotor.getEncoder();

        double kP = 1;
        double kI = 0;
        double kD = 0;
        pidController = new PIDController(kP, kI, kD);
    }

    /**
     * Move the arm to a desired position
     * @param position Target position in encoder ticks
     */
    public void moveToPosition(double position) {
        armMotor.set(pidController.calculate(armEncoder.getPosition(), position));
    }

    /**
     * Reset the encoder on the arm
     */
    public void resetEncoders() {
        armMotor.set(0);
        armEncoder.setPosition(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("position", armEncoder.getPosition());
    }
}
