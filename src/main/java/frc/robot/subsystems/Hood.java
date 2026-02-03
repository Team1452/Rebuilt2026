package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Simple PWM linear-actuator controller.
 *
 * Features:
 * - Open-loop PWM control via {@link edu.wpi.first.wpilibj.Servo} (0.0..1.0).
 * - Optional position feedback via a {@link DoubleSupplier} that returns position in centimeters.
 * - Convenience command factories (one-shot and wait-until-on-target when feedback is present).
 *
 * Notes / assumptions:
 * - If you provide a feedback supplier, it must return the actuator position in the same
 *   units as the min/max meters passed to the constructor (meters by convention here).
 * - If no feedback supplier is provided, commands are open-loop (they issue a PWM setpoint
 *   but cannot detect when the mechanism is on-target).
 */
public class Hood extends SubsystemBase {
  private final PWMSparkMax actuator;
  private final boolean hasFeedback;
  private final Encoder encoder;

  // Last commanded value (-1..1) used to estimate position when feedback is absent
  private double lastCommanded = 0.0;

  /**
   * Create an open-loop linear actuator controller on the given PWM channel.
   *
   * @param pwmChannel PWM channel for the actuator (port number)
   */

  public Hood(int pwmChannel) {
    this.actuator = new PWMSparkMax(pwmChannel);
    this.hasFeedback = false;
    this.encoder = new Encoder(0, 1); 

    this.encoder.setDistancePerPulse(0.1); // Example: 0.1 cm per pulse
  }

  public void setPosition(double position) {
    // PWMSparkMax is a motor controller (range -1..1). We clamp the input here.
    double v = MathUtil.clamp(position, -1.0, 1.0);
    actuator.set(v);
    lastCommanded = v;
  }

  public void setSpeed(double speed) {
    double v = MathUtil.clamp(speed, -1.0, 1.0);
    actuator.set(v);
    lastCommanded = v;
  }
  
  public double getPositionCentimeters() {
    // PWMSparkMax doesn't provide position feedback. Return last commanded value as an
    // estimate (units are motor output, not centimeters) to preserve callers.
    return lastCommanded;
  }

  public Command setPositionCommand(double position) {
    System.out.println("Setting hood position to: " + position);
    return Commands.runOnce(() -> setPosition(position), this);
  }

  public Command setSpeedCommand(double speed) {  
    System.out.println("Setting hood speed to: " + speed);
    return Commands.runOnce(() -> setSpeed(speed), this);
  }

@Override
  public void periodic() {
    System.out.println("Hood Position: " + getPositionCentimeters() + " cm");
  }

}
