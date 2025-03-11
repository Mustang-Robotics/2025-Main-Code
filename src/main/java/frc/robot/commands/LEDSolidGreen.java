package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED;

public class LEDSolidGreen extends Command {
    private final LED m_led;
    public LEDSolidGreen(LED led){
        m_led = led;

        addRequirements(m_led);
    }

    @Override
    public void initialize(){
        m_led.SolidGreen();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}