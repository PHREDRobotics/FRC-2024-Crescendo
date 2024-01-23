package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TestConstants;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

/**
 *  Arm Subsystem for PHRED's really cool robot
 */
public class ArmSubsystem extends SubsystemBase {
  
    private CANSparkMax armMotor = new CANSparkMax(TestConstants.kTestMotorCanId, MotorType.kBrushless);
    SparkPIDController pidController = armMotor.getPIDController();
    RelativeEncoder armEncoder = armMotor.getEncoder();
    
    public ArmSubsystem() {
        pidSetup();
    }

    public void pidSetup() {
        pidController.setFeedbackDevice(armEncoder);

        pidController.setP(0.01);
        pidController.setI(0.001);
        pidController.setD(0.001);
    }

    /**
     * Move the arm to a desired position
     * @param position 5 = low, 20 = mid, 30 = high
     */
    public void moveToPosition(double position) {
        pidController.setReference(position, CANSparkMax.ControlType.kPosition);
    }
}
