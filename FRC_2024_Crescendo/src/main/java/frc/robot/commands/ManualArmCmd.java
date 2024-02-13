package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

/**
 * Command for manually moving the arm
 */
public class ManualArmCmd extends Command {

    DoubleSupplier armPower;
    ArmSubsystem arm_subsystem;

    /**
     * 
     * @param leftLiftPower power for the left lift motor
     * @param rightLiftPower power for the right lift motor
     * @param liftSubsystem
     */
    public ManualArmCmd(DoubleSupplier ArmPower, ArmSubsystem armSubsystem) {
        this.armPower = ArmPower;
        this.arm_subsystem = armSubsystem;

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        arm_subsystem.resetEncoders();
    }

    @Override
    public void execute() {
        this.arm_subsystem.setRawPower(this.armPower);
    }
}
