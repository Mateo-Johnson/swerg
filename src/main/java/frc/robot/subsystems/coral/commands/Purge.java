package frc.robot.subsystems.coral.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.Coral;

public class Purge extends Command {
    private final Coral coral;
    private final double speed;
    
    /**
     * Command to move the Coral subsystem backward
     * 
     * @param coral The Coral subsystem
     * @param speed The speed at which to move the wheels
     */
    public Purge(Coral coral, double speed) {
        this.coral = coral;
        this.speed = speed;
        addRequirements(coral);
    }
    
    @Override
    public void initialize() {
        // Start moving backwards
        coral.reverse(speed);
    }
    
    @Override
    public void execute() {
        // Command runs continuously until interrupted
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop motors
        coral.stop();
    }
    
    @Override
    public boolean isFinished() {
        // This command runs until interrupted
        return false;
    }
}