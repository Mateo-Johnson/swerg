package frc.robot.subsystems.elevator.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorState;

public class MoveToPoint extends Command {
    private final Elevator elevator;
    private final double targetPosition;

    public MoveToPoint(Elevator elevator, double targetPosition) {
        this.elevator = elevator;
        this.targetPosition = targetPosition;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setPosition(targetPosition);
    }

    @Override
    public void execute() {
        // Position control is handled in the elevator's periodic method
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            elevator.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return elevator.getCurrentState() == ElevatorState.AT_SETPOINT;
    }
}