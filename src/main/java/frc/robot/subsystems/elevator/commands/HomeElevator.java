package frc.robot.subsystems.elevator.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class HomeElevator extends Command {
    private final Elevator m_elevator;

    public HomeElevator(Elevator elevator, int setpointIndex) {
        m_elevator = elevator;
        addRequirements(m_elevator);
    }

    @Override
    public void initialize() {
        m_elevator.startHoming();
    }

    @Override
    public boolean isFinished() {
        return m_elevator.atSetpoint();
    }
}