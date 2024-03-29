package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

/**
 * A command for moving the arm
 */
public class ArmMotor extends Command {
    private final ArmSubsystem m_arm;
    private final Double m_arm_pos;

    /**
     * Operate the arm subsystem
     * @param armPos Default positions can be found in the Constants
     * @param arm 
     */
    public ArmMotor(Double armPos, ArmSubsystem arm) {
        m_arm_pos = armPos;
        m_arm = arm;
        addRequirements(m_arm);
    }

    @Override
    public void execute() {
        m_arm.moveToPosition(m_arm_pos);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted == false) {
            m_arm.moveToPosition(Constants.ArmConstants.kArmLow);
        }
    }
}
