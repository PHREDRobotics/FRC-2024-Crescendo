package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ResetArmEncoder extends Command {
    
    private ArmSubsystem m_arm_subsystem;


    public ResetArmEncoder(ArmSubsystem armSubsystem) {
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
