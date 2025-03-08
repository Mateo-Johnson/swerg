package frc.robot.subsystems.coral.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.Coral;

public class Intake extends Command {
    private final Coral coral;
    private final double speed;
    private final boolean stopOnDetection;
    
    /**
     * Command to move the Coral subsystem forward for intake
     * 
     * @param coral The Coral subsystem
     * @param speed The speed at which to move the wheels
     */
    public Intake(Coral coral, double speed) {
        this(coral, speed, false);
    }
    
    /**
     * Command to move the Coral subsystem forward for intake
     * 
     * @param coral The Coral subsystem
     * @param speed The speed at which to move the wheels
     * @param stopOnDetection Whether to stop the command when a game piece is detected
     */
    public Intake(Coral coral, double speed, boolean stopOnDetection) {
        this.coral = coral;
        this.speed = speed;
        this.stopOnDetection = stopOnDetection;
        addRequirements(coral);
    }
    
    @Override
    public void initialize() {
        // Start moving forward
        coral.forward(speed);
    }
    
    @Override
    public void execute() {
        // Command runs continuously until interrupted or game piece detected
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop motors
        coral.stop();
    }
    
    @Override
    public boolean isFinished() {
        // This command runs until interrupted or game piece detected (if stopOnDetection is true)
        return stopOnDetection && coral.hasGamePiece();
    }
}