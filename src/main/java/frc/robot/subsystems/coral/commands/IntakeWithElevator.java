package frc.robot.subsystems.coral.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.coral.Coral;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.commands.ElevatorZero;

public class IntakeWithElevator extends SequentialCommandGroup {
    
    /**
     * Creates a command sequence that first moves the elevator to its lower limit (zero position),
     * then runs the intake once the elevator is fully lowered
     * 
     * @param coral The Coral subsystem for intake
     * @param elevator The Elevator subsystem
     * @param intakeSpeed The speed at which to run the intake
     * @param elevatorSpeed The speed at which to move the elevator down
     */
    public IntakeWithElevator(Coral coral, Elevator elevator, double intakeSpeed, double elevatorSpeed) {
        addCommands(
            // First move the elevator down until it hits lower limit
            new ElevatorZero(elevator, elevatorSpeed),
            
            // Then start the intake wheels after elevator is zeroed
            new Intake(coral, intakeSpeed)
        );
    }
}