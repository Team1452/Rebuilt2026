package frc.robot.subsystems;

<<<<<<< HEAD
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
=======
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.VBatOutputModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.lang.reflect.Method;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * CANdleExample - a minimal, safe CANdle example that compiles with Phoenix 6.
 *
 * This class intentionally avoids directly referencing specific animation classes
 * so it remains compatible across Phoenix 6 versions. At runtime it will try to
 * use reflection to call whichever methods are available in the CTRE library in
 * your build (for example: setLEDs(...) or setControl(<Animation>)).
 *
 * Usage:
 * - Construct with the CAN device ID in `Constants.CANdleID`.
 * - Call `setSolidColor(...)` to set a static color.
 * - Call `startAnimation("Rainbow")` or `startAnimation("Larson")` to attempt
 *   to start a named animation if it exists in your Phoenix 6 JAR.
 *
 * Notes:
 * - If you prefer direct compilation against animation classes, replace the
 *   reflection parts with concrete types from your Phoenix 6 version.
 * - Phoenix Tuner has built-in examples and a live editor; you can use the
 *   tuner to prototype animations and then port parameters into this class.
 */
public class CANdleExample extends SubsystemBase {
  private static final Logger logger = Logger.getLogger(CANdleExample.class.getName());

  private final CANdle m_candle;
  private static final int DEFAULT_CAN_ID = 5; // change to your CANdle device ID if needed
  private final Timer m_timer = new Timer();

  public CANdleExample() {
  m_candle = new CANdle(DEFAULT_CAN_ID, "rio");

    CANdleConfiguration cfg = new CANdleConfiguration();
    cfg.LED.StripType = StripTypeValue.GRB;
    cfg.LED.BrightnessScalar = 0.12; // conservative default
    cfg.CANdleFeatures.VBatOutputMode = VBatOutputModeValue.Modulated;

    // apply settings reflectively to tolerate API differences between Phoenix 6 versions
    try {
      try {
        Method cfgMethod = CANdle.class.getMethod("configAllSettings", CANdleConfiguration.class, int.class);
        cfgMethod.invoke(m_candle, cfg, Integer.valueOf(100));
      } catch (NoSuchMethodException ns) {
        // fallback to single-arg signature if present
        try {
          Method cfgMethod2 = CANdle.class.getMethod("configAllSettings", CANdleConfiguration.class);
          cfgMethod2.invoke(m_candle, cfg);
        } catch (NoSuchMethodException ns2) {
          logger.info("CANdle.configAllSettings not found with expected signatures; skipping configAllSettings call.");
        }
      }
    } catch (Exception e) {
      logger.log(Level.WARNING, "Failed to config CANdle settings reflectively", e);
    }

    m_timer.start();
  }

  /**
   * Set a solid color on the strip.
   * This tries to call a few common setLEDs signatures reflectively; if none are
   * available it will fall back to attempting to wrap a simple Static animation
   * and call setControl reflectively.
   */
  public void setSolidColor(int r, int g, int b, int w, int startIndex, int length) {
    // Try common method: setLEDs(int r, int g, int b, int w, int start, int len)
    try {
      Method m = CANdle.class.getMethod("setLEDs", int.class, int.class, int.class, int.class, int.class, int.class);
      m.invoke(m_candle, r, g, b, w, startIndex, length);
      return;
    } catch (NoSuchMethodException ignored) {
      // try other signatures below
    } catch (Exception e) {
      logger.log(Level.WARNING, "Invocation of setLEDs(r,g,b,w,...) failed", e);
      return;
    }

    // Try alternate signature: setLEDs(int[] rawBytes, int start)
    try {
      Method alt = CANdle.class.getMethod("setLEDs", byte[].class, int.class);
      // Build RGBW byte array (R,G,B,W repeated)
      byte[] raw = new byte[length * 4];
      for (int i = 0; i < length; ++i) {
        raw[i * 4 + 0] = (byte) r;
        raw[i * 4 + 1] = (byte) g;
        raw[i * 4 + 2] = (byte) b;
        raw[i * 4 + 3] = (byte) w;
      }
      alt.invoke(m_candle, (Object) raw, startIndex);
      return;
    } catch (NoSuchMethodException ignored) {
    } catch (Exception e) {
      logger.log(Level.FINER, "Invocation of setLEDs(byte[],start) failed", e);
      return;
    }

    // Last resort: try to create a simple static animation/control object reflectively
    try {
      // Example animation class names to try
      String[] candidate = {"StaticAnimation", "SolidAnimation", "SetAllAnimation"};
      Object anim = null;
      for (String name : candidate) {
        try {
          Class<?> cls = Class.forName("com.ctre.phoenix6.controls." + name);
          // try default constructor
          anim = cls.getConstructor().newInstance();
          break;
        } catch (ClassNotFoundException e) {
          // try next
        }
      }
      if (anim != null) {
        // attempt to set color fields if present (reflectively)
        try {
          var fR = anim.getClass().getField("Red");
          fR.setInt(anim, r);
        } catch (Exception ignored) {}
        try {
          var fG = anim.getClass().getField("Green");
          fG.setInt(anim, g);
        } catch (Exception ignored) {}
        try {
          var fB = anim.getClass().getField("Blue");
          fB.setInt(anim, b);
        } catch (Exception ignored) {}

        // find a setControl method and invoke it
        Method setControl = findSetControlMethod();
        if (setControl != null) {
          setControl.invoke(m_candle, anim);
          return;
        }
      }
    } catch (Exception e) {
      logger.log(Level.FINER, "Fallback static animation attempt failed", e);
    }

    logger.info("Could not set solid color: no compatible CANdle method found in this Phoenix 6 JAR.");
  }

  /**
   * Try to start a named animation (e.g. "Rainbow", "Larson", "Twinkle").
   * This method will attempt to instantiate an animation class named
   * com.ctre.phoenix6.controls.<name>Animation using reflection and pass it to
   * m_candle.setControl(...) if possible.
   */
  public void startAnimation(String name) {
    String className = "com.ctre.phoenix6.controls." + name + "Animation";
    try {
      Class<?> cls = Class.forName(className);
      Object anim = null;
      // try a variety of constructors: default, (startIdx,endIdx), or (frameRate, ...)
      try { anim = cls.getConstructor().newInstance(); } catch (Exception ignored) {}
      if (anim == null) {
        try { anim = cls.getConstructor(int.class, int.class).newInstance(0, 31); } catch (Exception ignored) {}
      }
      if (anim == null) {
        // try single-int constructor
        try { anim = cls.getConstructor(int.class).newInstance(30); } catch (Exception ignored) {}
      }
      if (anim == null) {
        logger.info("Could not construct animation instance for: " + className);
        return;
      }

      // Attempt to tune some common fields reflectively (FrameRate, Color, Size)
      try {
        var f = anim.getClass().getField("FrameRate");
        f.setDouble(anim, 30.0);
      } catch (Exception ignored) {}

      try {
        var f = anim.getClass().getField("Size");
        f.setInt(anim, 4);
      } catch (Exception ignored) {}

      Method setControl = findSetControlMethod();
      if (setControl != null) {
        setControl.invoke(m_candle, anim);
      } else {
        logger.info("No setControl method found to start animation");
      }
    } catch (ClassNotFoundException e) {
      logger.info("Animation class not found: " + className);
    } catch (Exception e) {
      logger.log(Level.WARNING, "Failed to start animation " + name, e);
    }
  }

  /**
   * Clear any active animation / return to default static state.
   */
  public void clear() {
    // Try to call a "clearAnimation" or send null to setControl
    try {
      Method setControl = findSetControlMethod();
      if (setControl != null) {
        setControl.invoke(m_candle, new Object[] { null });
        return;
      }
    } catch (Exception ignored) {}

    logger.info("clear(): no compatible clear method on CANdle found");
  }

  private Method findSetControlMethod() {
    for (Method m : CANdle.class.getMethods()) {
      if (!m.getName().equals("setControl")) continue;
      // prefer methods with a single parameter
      if (m.getParameterCount() == 1) return m;
    }
    return null;
  }

  @Override
  public void periodic() {
    // Example: blink a single LED white once per second for diagnostics
    double t = m_timer.get();
    if (((int) t) % 2 == 0) {
      setSolidColor(255, 255, 255, 0, 0, 1);
    } else {
      setSolidColor(0, 0, 0, 0, 0, 1);
    }
  }
}
>>>>>>> refs/remotes/origin/main
