package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;

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
  private boolean isUppies = false;
  private boolean isDownies = false;

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
    double v = MathUtil.clamp(position, 0.025, 0.6);
    actuator.set(v);
    lastCommanded = v;
  }

  public void uppies() {
    isUppies = true;
    isDownies = false;
  }

  public void downies() {
    isDownies = true;
    isUppies = false;
  }

  public void neutral() {
    isUppies = false;
    isDownies = false;
  }
  
  public double getPositionCentimeters() {
    // PWMSparkMax doesn't provide position feedback. Return last commanded value as an
    // estimate (units are motor output, not centimeters) to preserve callers.
    return lastCommanded;
  }

  public Command setPositionCommand(double position) {
    return Commands.runOnce(() -> setPosition(position), this);
  }

  public Command up() {
    return Commands.run(() -> uppies(), this);
  }

  public Command down() {
    return Commands.run(() -> downies(), this);
  }

  public Command neutralCommand() {
    return Commands.run(() -> neutral(), this);
  }

  public Command goFlat() {
    return Commands.runOnce(() -> setPosition(0.025), this);
  }

  public Command constantUpdateCommand(Drive drive) {
    final Translation2d blueHopper = new Translation2d(4.623, 4.01);

    DoubleSupplier distance = () -> Math.hypot((blueHopper.minus(drive.getPose().getTranslation())).getX(), (blueHopper.minus(drive.getPose().getTranslation())).getY());

    //DoubleSupplier position = () -> -0.378 + 5.96E-03*distance.getAsDouble() + 8.5E-03*Math.pow(distance.getAsDouble(),2) + -6E-04*Math.pow(distance.getAsDouble(),3) + 1.18E-05*Math.pow(distance.getAsDouble(),4);

    lastCommanded = distance.getAsDouble();
    return Commands.run(() -> setPositionCommand(distance.getAsDouble()));
  } 

@Override
  public void periodic() {
    System.out.println(lastCommanded);

    if (isUppies) {
      lastCommanded += 0.005;
      setPosition(lastCommanded);
    } else if (isDownies) {
      lastCommanded -= 0.005;
      setPosition(lastCommanded);
    }

    Logger.recordOutput("Hood/Position", getPositionCentimeters());

  }

}
