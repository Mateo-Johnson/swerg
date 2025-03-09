package frc.robot.subsystems.coral.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.Coral;

public class Intake extends Command {
    private final Coral coral;
    private final double speed;
    private boolean isEjecting = false;
    
    /**
     * Command to move the Coral subsystem.
     * First execution: Intakes until a game piece is detected
     * Second execution: Continues intaking without checking for game piece detection
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
        // Check if the subsystem already has a game piece
        if (coral.hasGamePiece() && !isEjecting) {
            // We have a game piece and haven't started ejecting yet, so this is an eject operation
            isEjecting = true;
            // Use same direction for both operations
            coral.forward(speed);
        } else if (isEjecting) {
            // We were ejecting and command was called again, reset to intake mode
            isEjecting = false;
            // Stop motors since we're done with the full cycle
            coral.stop();
        } else {
            // Normal intake operation
            isEjecting = false;
            coral.forward(speed);
        }
    }
    
    @Override
    public void execute() {
        // Command runs continuously until interrupted or finished
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop motors
        coral.stop();
        
        // If we were ejecting and got interrupted, reset the state
        if (interrupted && isEjecting) {
            isEjecting = false;
        }
    }
    
    @Override
    public boolean isFinished() {
        if (isEjecting) {
            // Continue running when in ejection mode until manually interrupted
            return false;
        } else {
            // When intaking, finish when a game piece is detected
            return coral.hasGamePiece();
        }
    }
}