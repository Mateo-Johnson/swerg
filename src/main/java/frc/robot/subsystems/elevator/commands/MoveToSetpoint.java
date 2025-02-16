package frc.robot.subsystems.elevator.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class MoveToSetpoint extends Command {
    private final Elevator m_elevator;
    private final int m_setpointIndex;

    public MoveToSetpoint(Elevator elevator, int setpointIndex) {
        m_elevator = elevator;
        m_setpointIndex = setpointIndex;
        addRequirements(m_elevator);
    }

    @Override
    public void initialize() {
        m_elevator.moveToSetpoint(m_setpointIndex);
    }

    @Override
    public boolean isFinished() {
        return m_elevator.atSetpoint();
    }
}