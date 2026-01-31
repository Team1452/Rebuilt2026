package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.hardware.CANdle;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;


public class LEDSubsystem extends SubsystemBase {
    private final  int CANdleID = 5;


    /* color can be constructed from RGBW, a WPILib Color/Color8Bit, HSV, or hex */
    private static final RGBWColor kGreen = new RGBWColor(0, 217, 0, 0);
    private static final RGBWColor kWhite = new RGBWColor(Color.kWhite).scaleBrightness(0.5);
    private static final RGBWColor kViolet = RGBWColor.fromHSV(Degrees.of(270), 0.9, 0.8);
    private static final RGBWColor kRed = RGBWColor.fromHex("#D9000000").orElseThrow();

    /*
     * Start and end index for LED animations.
     * 0-7 are onboard, 8-66 are an external strip.
     * CANdle supports 8 animation slots (0-7).
     */

    private static final int kSlot0StartIdx = 0;
    private static final int kSlot0EndIdx = 7;

    private static final int kSlot1StartIdx = 8;
    private static final int kSlot1EndIdx = 66;

    private final CANdle m_candle = new CANdle(CANdleID, TunerConstants.kCANBus.getName());
    
    private AnimationType m_anim0State = AnimationType.None;
    private AnimationType m_anim1State = AnimationType.None;

    private final SendableChooser<AnimationType> m_anim0Chooser = new SendableChooser<AnimationType>();
    private final SendableChooser<AnimationType> m_anim1Chooser = new SendableChooser<AnimationType>();


    public LEDSubsystem() {
        var cfg = new CANdleConfiguration();
        cfg.LED.StripType = StripTypeValue.GRB;
        cfg.LED.BrightnessScalar = 0.5;
        m_candle.getConfigurator().apply(cfg);
        /* clear all previous animations */
        for (int i = 0; i < 8; ++i) {
            m_candle.setControl(new EmptyAnimation(i));
        }
        /* set the onboard LEDs to a solid color */
        m_candle.setControl(new SolidColor(0, 3).withColor(kGreen));
        m_candle.setControl(new SolidColor(4, 7).withColor(kWhite));

        //seems to assign different animation types for different sections of the lights.
        /* add animations to chooser for slot 0 */
        m_anim0Chooser.setDefaultOption("Color Flow", AnimationType.ColorFlow);
        m_anim0Chooser.addOption("Rainbow", AnimationType.Rainbow);
        m_anim0Chooser.addOption("Twinkle", AnimationType.Twinkle);
        m_anim0Chooser.addOption("Twinkle Off", AnimationType.TwinkleOff);
        m_anim0Chooser.addOption("Fire", AnimationType.Fire);

        /* add animations to chooser for slot 1 */
        m_anim1Chooser.setDefaultOption("Larson", AnimationType.Larson);
        m_anim1Chooser.addOption("RGB Fade", AnimationType.RgbFade);
        m_anim1Chooser.addOption("Single Fade", AnimationType.SingleFade);
        m_anim1Chooser.addOption("Strobe", AnimationType.Strobe);
        m_anim1Chooser.addOption("Fire", AnimationType.Fire);

        SmartDashboard.putData("Animation 0", m_anim0Chooser);
        SmartDashboard.putData("Animation 1", m_anim1Chooser);
    }



    public void setAnimation(AnimationType type, int slot) {
        int startIdx, endIdx;//gets start location and end location for different light sections
        if (slot == 0) {
            startIdx = kSlot0StartIdx;
            endIdx = kSlot0EndIdx;
        } else {
            startIdx = kSlot1StartIdx;
            endIdx = kSlot1EndIdx;
        }


        switch (type) {
            // Still need to add all animation types,is incomplete as is
                default:
                case ColorFlow:
                    m_candle.setControl(
                        new ColorFlowAnimation(startIdx, endIdx).withSlot(slot)
                            .withColor(kViolet)
                    );
                    break;
                case Rainbow:
                    m_candle.setControl(
                        new RainbowAnimation(startIdx, endIdx).withSlot(slot)
                    );
                    break;
                case Twinkle:
                    m_candle.setControl(
                        new TwinkleAnimation(startIdx, endIdx).withSlot(slot)
                            .withColor(kViolet)
                    );
                    break;
                case TwinkleOff:
                    m_candle.setControl(
                        new TwinkleOffAnimation(startIdx, endIdx).withSlot(slot)
                            .withColor(kViolet)
                    );
                    break;
                case Fire:
                    m_candle.setControl(
                        new FireAnimation(startIdx, endIdx).withSlot(slot)
                    );
                    break;
                case Larson:
                    m_candle.setControl(
                        new LarsonAnimation(startIdx, endIdx).withSlot(slot)
                            .withColor(kViolet)
                    );
                    break;
        }
    }



    @Override
    public void periodic() {
        
        /* if the selection for slot 0 changes, change animations */
        final var anim0Selection = m_anim0Chooser.getSelected();
        if (anim0Selection != null && m_anim0State != anim0Selection) {
            m_anim0State = anim0Selection;

            setAnimation(m_anim0State, 0);
        }

        /* if the selection for slot 1 changes, change animations */
        final var anim1Selection = m_anim1Chooser.getSelected();
        if (anim1Selection != null && m_anim1State != anim1Selection) {
            m_anim1State = anim1Selection;

            setAnimation(m_anim1State, 1);
            
        }
    }

    // Can add methods to set animations directly-can incoroprate with robot container on button press logic
    public void setAnimation(AnimationType type) {
        // Logic to switch the CANdle animation
    }
}