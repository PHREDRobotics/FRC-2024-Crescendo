package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

/**
 * Arm Subsystem that controls arm PID for PHRED's really cool robot
 */
public class ArmSubsystem extends SubsystemBase {

    private int m_can_id;
    private MotorType m_motor_type;
    public RelativeEncoder armEncoder;
    public CANSparkMax armMotor;
    private SparkPIDController pidController;

    public ArmSubsystem(int canID, MotorType motorType) {
        m_can_id = canID;
        m_motor_type = motorType;

        armMotor = new CANSparkMax(m_can_id, m_motor_type);
        armEncoder = armMotor.getEncoder();
        pidController = armMotor.getPIDController();

        pidController.setFeedbackDevice(armEncoder);

        pidController.setP(0.9);
        pidController.setI(0);
        pidController.setD(0);
    }

    /**
     * Move the arm to a desired position
     * 
     * @param position 5 = low, 20 = mid, 30 = high
     */
    public void moveToPosition(double position) {
        armMotor.set(0.1);

        /*
         *  pidController.setReference(position, CANSparkMax.ControlType.kPosition);
         */
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("position", armEncoder.getPosition());
    }
}
