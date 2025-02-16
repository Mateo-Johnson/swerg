// ManualElevatorCommand.java
package frc.robot.subsystems.elevator.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorState;

public class ManualElevatorCommand extends Command {
    private final Elevator m_elevator;
    private final double m_speed;

    public ManualElevatorCommand(Elevator elevator, double speed) {
        m_elevator = elevator;
        m_speed = speed;
        addRequirements(m_elevator);
    }

    @Override
    public void initialize() {
        m_elevator.setManualControl(true);
    }

    @Override
    public void execute() {
        m_elevator.setManualSpeed(m_speed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_elevator.setManualControl(false);
        m_elevator.setState(ElevatorState.IDLE);
    }
}