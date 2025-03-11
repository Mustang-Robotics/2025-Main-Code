package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;

public class IntakeCurrentStop extends Command {
    private final Intake m_intake;
    private final double m_speed;
    private final LED m_led;


    public IntakeCurrentStop(Intake intake, LED led, double speed) {
        m_intake = intake;
        m_led = led;
        m_speed = speed;
        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
        m_intake.setIntakeSpeed(m_speed);
    }

    @Override
    public boolean isFinished() {
        return m_intake.getIntakeCurrent() > 45;
    }

    @Override
    public void end(boolean interupted) {
        m_intake.setIntakeSpeed(0);
    }

}
