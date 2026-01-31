package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.Utils;

public class CANdleExample extends SubsystemBase {
    // 1. Define the hardware and constants at the class level
    private final int kCANdleId = 0;
    private final CANdle m_candle = new CANdle(kCANdleId, "CANivore 1 - testbed");

    // 2. Define the animations
    private final SingleFadeAnimation m_fade = new SingleFadeAnimation(0, 7);
    private final RainbowAnimation m_rainbow = new RainbowAnimation(8, 67);

    public CANdleExample() {
        // 3. Configure the CANdle
        CANdleConfiguration configs = new CANdleConfiguration();

        configs.LED.StripType = StripTypeValue.GRB;
        configs.LED.BrightnessScalar = 0.5;
        configs.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;

        m_candle.getConfigurator().apply(configs);

        // 4. Start the animations
        // Slot 0 for the green fade (LEDs 0-7)
        m_candle.setControl(m_fade); 
        // Slot 1 for the rainbow (LEDs 8-67)
        m_candle.setControl(m_rainbow.withSlot(1));
    }

    // You can add methods here to change colors later!
  
}