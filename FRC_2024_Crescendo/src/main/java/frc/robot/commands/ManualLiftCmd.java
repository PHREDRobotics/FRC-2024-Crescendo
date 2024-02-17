package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LiftSubsystem;

/**
 * Command for manually lifting the robot
 */
public class ManualLiftCmd extends Command {

    DoubleSupplier left_power;
    DoubleSupplier right_power;
    LiftSubsystem lift_subsystem;

    /**
     * 
     * @param leftLiftPower power for the left lift motor
     * @param rightLiftPower power for the right lift motor
     * @param liftSubsystem
     */
    public ManualLiftCmd(DoubleSupplier leftLiftPower, DoubleSupplier rightLiftPower, LiftSubsystem liftSubsystem) {
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
        this.lift_subsystem.setRawLeftPower(this.left_power);
        this.lift_subsystem.setRawRightPower(this.right_power);
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
