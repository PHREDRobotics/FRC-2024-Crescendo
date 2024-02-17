package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

/**
 * Command to reset the encoder on the arm
 */
public class ManualResetArmEncoder extends Command {

    private ArmSubsystem m_arm_subsystem;

    public ManualResetArmEncoder(ArmSubsystem armSubsystem) {
        m_arm_subsystem = armSubsystem;
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        m_arm_subsystem.resetEncoders();
    }
}
