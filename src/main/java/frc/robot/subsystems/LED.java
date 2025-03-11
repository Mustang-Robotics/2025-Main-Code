package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
    private final AddressableLED m_led = new AddressableLED(0);
    private final AddressableLEDBuffer m_buffer = new AddressableLEDBuffer(36);
    private final AddressableLEDBufferView m_right = m_buffer.createView(0,17);
    private final AddressableLEDBufferView m_left = m_buffer.createView(18, 35).reversed();
    private LEDPattern red = LEDPattern.solid(Color.kGreen);
    private LEDPattern blue = LEDPattern.solid(Color.kBlue);
    private LEDPattern green = LEDPattern.solid(Color.kRed);
    private LEDPattern blink_green = green.blink(Seconds.of(.0625));
    private LEDPattern blink_blue = blue.blink(Seconds.of(.0625));
    private LEDPattern blink_red = red.blink(Seconds.of(.0625));

    public LED (){
        m_led.setLength(m_buffer.getLength());
        m_led.setData(m_buffer);
        m_led.start();
    }

    public void SolidGreen(){
        green.applyTo(m_buffer);
        m_led.setData(m_buffer);
    }

    public void SolidRed(){
        red.applyTo(m_buffer);
        m_led.setData(m_buffer);
    }

    public void SolidBlue(){
        blue.applyTo(m_buffer);
        m_led.setData(m_buffer);
    }

    public void BlinkGreen(){
        blink_green.applyTo(m_buffer);
        m_led.setData(m_buffer);
    }

    public void BlinkBlue(){
        blink_blue.applyTo(m_buffer);
        m_led.setData(m_buffer);
    }

    public void BlinkRed(){
        blink_red.applyTo(m_buffer);
        m_led.setData(m_buffer);
    }
}
