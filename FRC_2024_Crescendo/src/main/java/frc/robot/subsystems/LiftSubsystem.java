package frc.robot.subsystems;
<<<<<<< HEAD
=======

import java.util.function.DoubleSupplier;
>>>>>>> 626e5c332d4f3aec8f29559f5e3a8eb5961e7253

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CAN;
<<<<<<< HEAD
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
=======
import edu.wpi.first.wpilibj.SPI;
>>>>>>> 626e5c332d4f3aec8f29559f5e3a8eb5961e7253
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LiftSubsystem extends SubsystemBase {

    private CANSparkMax leftLiftMotor;
    private CANSparkMax rightLiftMotor;
    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;
<<<<<<< HEAD
    private AHRS navXMicro;
=======
    private AHRS gyro;
>>>>>>> 626e5c332d4f3aec8f29559f5e3a8eb5961e7253

    public LiftSubsystem() {
        leftLiftMotor = new CANSparkMax(Constants.LiftConstants.kLeftLiftControllerPort, MotorType.kBrushless);
        rightLiftMotor = new CANSparkMax(Constants.LiftConstants.kRightLiftControllerPort, MotorType.kBrushless);
        leftEncoder = leftLiftMotor.getEncoder();
        rightEncoder = rightLiftMotor.getEncoder();
<<<<<<< HEAD
        try {
            navXMicro = new AHRS(SPI.Port.kMXP);
        } catch (RuntimeException ex) {
            DriverStation.reportError(ex.getMessage(), true);
        }
        Timer.delay(1.0);

        addChild("NavX Micro", navXMicro);
=======
        gyro = new AHRS(SPI.Port.kMXP);
>>>>>>> 626e5c332d4f3aec8f29559f5e3a8eb5961e7253
    }

    public void resetEncoders() {
        leftLiftMotor.set(0);
        rightLiftMotor.set(0);

        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

<<<<<<< HEAD
    public void retractLift() {
        leftLiftMotor.set(0.5 * (navXMicro.getRoll() / 90));
        rightLiftMotor.set(0.5 * (-navXMicro.getRoll() / 90));
    }

    public void extendLift() {
        leftLiftMotor.set(0);
        rightLiftMotor.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("NavX y", navXMicro.getPitch());
        SmartDashboard.putNumber("NavX x", navXMicro.getRoll());
        SmartDashboard.putNumber("NavX z", navXMicro.getYaw());
=======
    public void retractLeftLift() {
        leftLiftMotor.set(0.5 * (gyro.getRoll() / 90));
    }

    public void retractRightLift() {
        rightLiftMotor.set(0.5 * (-gyro.getRoll() / 90));
    }

    public void extendLeftLift() {
        leftLiftMotor.set(0);
    }

    public void extendRightLift() {
        rightLiftMotor.set(0);
    }

    public void setRawLeftPower(DoubleSupplier left_power) {
        leftLiftMotor.set(left_power.getAsDouble());
    }

    public void setRawRightPower(DoubleSupplier right_power) {
        rightLiftMotor.set(right_power.getAsDouble());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Gyro Yaw", gyro.getYaw());
>>>>>>> 626e5c332d4f3aec8f29559f5e3a8eb5961e7253
    }
}