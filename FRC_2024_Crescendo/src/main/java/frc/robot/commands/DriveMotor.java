package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import frc.robot.subsystems.MotorTestSubsystem;

public class DriveMotor extends Command {
    private final MotorTestSubsystem m_drivetrain;
    private final DoubleSupplier m_motor;


    public DriveMotor(DoubleSupplier motor, MotorTestSubsystem drivetrain) {
        m_motor = motor;
        m_drivetrain = drivetrain;
        addRequirements(m_drivetrain);
    }

    @Override
    public void execute() {
        m_drivetrain.drive(m_motor.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.drive(0);
    }
}
