package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LiftSubsystem;

public class ManualLiftCmd extends Command {

    DoubleSupplier left_power;
    DoubleSupplier right_power;
    LiftSubsystem lift_subsystem;

    public ManualLiftCmd(DoubleSupplier leftLiftPower, DoubleSupplier rightLiftPower, LiftSubsystem liftSubsystem) {
        this.left_power = leftLiftPower;
        this.right_power = rightLiftPower;
        this.lift_subsystem = liftSubsystem;

        addRequirements(liftSubsystem);
    }

    @Override
    public void execute() {
        this.lift_subsystem.setRawLeftPower(this.left_power);
        this.lift_subsystem.setRawRightPower(this.right_power);
    }
}
