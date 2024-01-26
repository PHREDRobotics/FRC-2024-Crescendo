package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ResetArmEncoder extends Command {
    
    private ArmSubsystem m_arm_subsystem;

    public ResetArmEncoder(ArmSubsystem armSubsystem) {
        m_arm_subsystem = armSubsystem;
    }

    @Override
    public void execute() {
        m_arm_subsystem.armMotor.set(0);
        m_arm_subsystem.armEncoder.setPosition(0);
    }
}
