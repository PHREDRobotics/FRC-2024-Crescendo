package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmMotor extends Command {
    private final ArmSubsystem m_arm;
    private final DoubleSupplier m_arm_pos;


    public ArmMotor(DoubleSupplier armPos, ArmSubsystem arm) {
        m_arm_pos = armPos;
        m_arm = arm;
        addRequirements(m_arm);
    }

    @Override
    public void execute() {
        m_arm.moveToPosition(m_arm_pos.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_arm.moveToPosition(5);
    }
}
