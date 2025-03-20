package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED;

public class LEDSolidPink extends Command {
    private final LED m_led;
    public LEDSolidPink(LED led){
        m_led = led;

        addRequirements(m_led);
    }

    @Override
    public void initialize(){
        m_led.SolidPink();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}