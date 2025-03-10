package frc.robot.subsystems.coral.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.coral.Coral;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.commands.MoveManual;

public class IntakeWithElevator extends ParallelCommandGroup {
    
    /**
     * Creates a command sequence that first moves the elevator down,
     * then runs the intake
     * 
     * @param coral The Coral subsystem for intake
     * @param elevator The Elevator subsystem
     * @param intakeSpeed The speed at which to run the intake
     * @param elevatorSpeed The speed at which to move the elevator down
     * @param elevatorTime How long to run the elevator down (in seconds)
     */
    public IntakeWithElevator(Coral coral, Elevator elevator, double intakeSpeed, double elevatorSpeed, double elevatorTime) {
        addCommands(
            // First move the elevator down for a set amount of time
            new MoveManual(elevator, elevatorSpeed).withTimeout(elevatorTime),
            
            // Then start the intake
            new Intake(coral, intakeSpeed)
        );
    }
}