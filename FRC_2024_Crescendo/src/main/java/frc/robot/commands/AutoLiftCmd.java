package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LiftSubsystem;

/**
 * Command for automatically lifting the robot
 */
public class AutoLiftCmd extends Command {

    LiftSubsystem lift_subsystem;

    public AutoLiftCmd(LiftSubsystem liftSubsystem) {
        this.lift_subsystem = liftSubsystem;

        addRequirements(liftSubsystem);
    }

    @Override
    public void execute() {
        this.lift_subsystem.AutoLift(0.5);
    }
}
