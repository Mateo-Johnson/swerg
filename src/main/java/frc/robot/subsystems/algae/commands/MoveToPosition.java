package frc.robot.subsystems.algae.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algae.Algae;

/**
 * Command to move the Algae subsystem to a specific position
 */
public class MoveToPosition extends Command {
    private final Algae m_algae;
    private final double m_position;
    
    /**
     * Creates a new MoveToPosition command
     * 
     * @param algae The Algae subsystem
     * @param position The target position to move to
     */
    public MoveToPosition(Algae algae, double position) {
        m_algae = algae;
        m_position = position;
        addRequirements(algae);
    }
    
    @Override
    public void initialize() {
        m_algae.setTargetPosition(m_position);
    }
    
    @Override
    public boolean isFinished() {
        return m_algae.getState() == Algae.STATES.AT_SETPOINT;
    }
}