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
        // Reset detection state when starting the command
        coral.resetGamePieceDetection();
        
        // Set initial state
        if (coral.hasGamePiece()) {
            currentState = State.EJECTING;
        } else {
            currentState = State.INTAKING;
        }
    }
    
    @Override
    public void execute() {
        // Command runs continuously until interrupted or finished
        switch (currentState) {
            case INTAKING:
                if (!coral.hasGamePiece()) {
                    coral.forward(speed);
                } else {
                    coral.stop();
                }
                break;
                
            case EJECTING:
                coral.reverse(speed);
                break;
                
            case IDLE:
            default:
                coral.stop();
                break;
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop motors
        coral.stop();
    }
    
    @Override
    public boolean isFinished() {
        // When intaking, finish when a game piece is detected
        // When ejecting, don't finish automatically (wait for button release)
        return currentState == State.INTAKING && coral.hasGamePiece();
    }
}