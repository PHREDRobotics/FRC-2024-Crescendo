package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LiftSubsystem extends SubsystemBase {

    private CANSparkMax leftLiftMotor;
    private CANSparkMax rightLiftMotor;
    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;
    private AHRS gyro;

    public LiftSubsystem() {
        leftLiftMotor = new CANSparkMax(Constants.LiftConstants.kLeftLiftControllerPort, MotorType.kBrushless);
        rightLiftMotor = new CANSparkMax(Constants.LiftConstants.kRightLiftControllerPort, MotorType.kBrushless);
        leftEncoder = leftLiftMotor.getEncoder();
        rightEncoder = rightLiftMotor.getEncoder();
        gyro = new AHRS(SPI.Port.kMXP);
    }

    public void resetEncoders() {
        leftLiftMotor.set(0);
        rightLiftMotor.set(0);

        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    public void retractLift() {
        leftLiftMotor.set(0.5 * (gyro.getRoll() / 180));
        rightLiftMotor.set(0.5 * (-gyro.getRoll() / 180));
    }

    public void extendLift() {
        leftLiftMotor.set(0);
        leftLiftMotor.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Gyro Yaw", gyro.getYaw());
    }
}