package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LiftSubsystem extends SubsystemBase {

    private CANSparkMax leftLiftMotor;
    private CANSparkMax rightLiftMotor;
    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;
    private AHRS navXMicro;

    public LiftSubsystem() {
        leftLiftMotor = new CANSparkMax(Constants.LiftConstants.kLeftLiftControllerPort, MotorType.kBrushless);
        rightLiftMotor = new CANSparkMax(Constants.LiftConstants.kRightLiftControllerPort, MotorType.kBrushless);
        leftEncoder = leftLiftMotor.getEncoder();
        rightEncoder = rightLiftMotor.getEncoder();
        try {
            navXMicro = new AHRS(SPI.Port.kMXP);
        } catch (RuntimeException ex) {
            DriverStation.reportError(ex.getMessage(), true);
        }
        Timer.delay(1.0);

        addChild("NavX Micro", navXMicro);
    }

    public void resetEncoders() {
        leftLiftMotor.set(0);
        rightLiftMotor.set(0);

        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    public void retractLeftLift() {
        leftLiftMotor.set(0.5 * (navXMicro.getRoll() / 90));
    }

    public void retractRightLift() {
        rightLiftMotor.set(0.5 * (-navXMicro.getRoll() / 90));
    }

    public void extendLeftLift() {
        if (leftEncoder.getPosition() == 0) {

            leftLiftMotor.set(0);

        } else {

            leftLiftMotor.set(.1);

        }
    }

    public void extendRightLift() {
                if (rightEncoder.getPosition() == 0) {

            rightLiftMotor.set(0);
            
        } else {

            rightLiftMotor.set(.1);

        }
    }

    public void setRawLeftPower(DoubleSupplier left_power) {
        leftLiftMotor.set(left_power.getAsDouble());
    }

    public void setRawRightPower(DoubleSupplier right_power) {
        rightLiftMotor.set(right_power.getAsDouble());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("NavX y", navXMicro.getPitch());
        SmartDashboard.putNumber("NavX x", navXMicro.getRoll());
        SmartDashboard.putNumber("NavX z", navXMicro.getYaw());
    }
}