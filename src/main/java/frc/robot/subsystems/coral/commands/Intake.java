package frc.robot.subsystems.coral.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.Coral;

public class Intake extends Command {
    private final Coral coral;
    private final double speed;
    private enum State { IDLE, INTAKING, EJECTING }
    private State currentState = State.IDLE;
    
    /**
     * Command to control the Coral subsystem.
     * First press: Intakes until a game piece is detected (then stops automatically)
     * Second press: Ejects the game piece (runs until button is released)
     * 
     * @param coral The Coral subsystem
     * @param speed The speed at which to move the wheels
     */
    public Intake(Coral coral, double speed) {
        this.coral = coral;
        this.speed = speed;
        addRequirements(coral);
    }
    
    @Override
    public void initialize() {
        // Determine what to do based on current state and game piece presence
        if (coral.hasGamePiece() && currentState != State.EJECTING) {
            // We have a game piece, start ejecting
            coral.forward(speed); // Same direction as intake
            currentState = State.EJECTING;
        } else if (!coral.hasGamePiece() && currentState != State.INTAKING) {
            // No game piece, start intaking
            coral.forward(speed);
            currentState = State.INTAKING;
        }
        // If we're already in the correct state, do nothing
    }
    
    @Override
    public void execute() {
        // Command runs continuously until interrupted or finished
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop motors
        coral.stop();
        
        // Reset state when done ejecting
        if (currentState == State.EJECTING) {
            currentState = State.IDLE;
        }
    }
    
    @Override
    public boolean isFinished() {
        // When intaking, finish when a game piece is detected
        // When ejecting, don't finish automatically (wait for button release)
        return currentState == State.INTAKING && coral.hasGamePiece();
    }
}