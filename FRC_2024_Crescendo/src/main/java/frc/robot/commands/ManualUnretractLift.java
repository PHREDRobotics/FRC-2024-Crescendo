package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LiftSubsystem;

/**
 * Command for manually lifting the robot
 */
public class ManualUnretractLift extends Command {

    BooleanSupplier left_power;
    BooleanSupplier right_power;
    LiftSubsystem lift_subsystem;

    /**
     * 
     * @param leftLiftPower power for the left lift motor
     * @param rightLiftPower power for the right lift motor
     * @param liftSubsystem
     */
    public ManualUnretractLift(BooleanSupplier leftLiftPower, BooleanSupplier rightLiftPower, LiftSubsystem liftSubsystem) {
        this.left_power = leftLiftPower;
        this.right_power = rightLiftPower;
        this.lift_subsystem = liftSubsystem;

        addRequirements(liftSubsystem);
    }

    @Override
    public void initialize() {
        lift_subsystem.resetEncoders();
    }

    @Override
    public void execute() {
        if(this.left_power.getAsBoolean()){
            this.lift_subsystem.setRawLeftPower(()->-0.1);
        } else {
            this.lift_subsystem.setRawLeftPower(()->0.0);
        }
        if(this.right_power.getAsBoolean()){
            this.lift_subsystem.setRawRightPower(()->-0.1);
        } else {
            this.lift_subsystem.setRawRightPower(()->0.0);
        }
    }

    @Override
    public boolean isFinished() {
        if (this.lift_subsystem.getLeftEncoder() > 100 || this.lift_subsystem.getRightEncoder() > 100) {
            return true;
        } else {
            return false;
        }
    }
}
