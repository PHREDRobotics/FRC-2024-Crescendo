/*package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LiftSubsystem extends SubsystemBase {
    AHRS gyro = new AHRS(SPI.Port.kMXP);
    
    double kP = 0;
    double kI = 0;
    double kD = 0;

    public LiftSubsystem() {
    }
    
    public Command raiseLift() {

        return runOnce(
            () -> {
                // Release Brakes
            });
        }
        
        public Command lowerLift() {
        
        return runOnce(
            () -> {
                // Move lift motors
            });
        }
        
        @Override
        public void periodic() {
            PIDController pidController = new PIDController(kP, kI, kD);
            

        }

}
*/