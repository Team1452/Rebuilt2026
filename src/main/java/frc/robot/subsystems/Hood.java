package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
<<<<<<< HEAD
=======
import frc.robot.generated.TunerConstants;

>>>>>>> nicky

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
<<<<<<< HEAD
  private final boolean hasFeedback;
  private final Encoder encoder;
=======
  //private final PWMSparkMax actuator2;
>>>>>>> nicky
  private final Drive drive;

  // Last commanded value (-1..1) used to estimate position when feedback is absent
  private double lastCommanded = 0.0;
  private boolean isUppies = false;
  private boolean isDownies = false;
  private boolean isDistanceControl = false;

  /**
   * Create an open-loop linear actuator controller on the given PWM channel.
   *
   * @param pwmChannel PWM channel for the actuator (port number)
   */

<<<<<<< HEAD
  public Hood(int pwmChannel, Drive drive) {
    this.actuator = new PWMSparkMax(pwmChannel);
    this.hasFeedback = false;
    this.encoder = new Encoder(0, 1); 

    this.encoder.setDistancePerPulse(0.1); // Example: 0.1 cm per pulse
=======
  public Hood(Drive drive) {
    this.actuator = new PWMSparkMax(TunerConstants.Hood1PWMChannel);
    //this.actuator2 = new PWMSparkMax(TunerConstants.Hood2PWMChannel);
>>>>>>> nicky
    this.drive = drive;
  }

  public void setPosition(double position) {
<<<<<<< HEAD
    double v = MathUtil.clamp(position, -1, 1);
=======
    double v = MathUtil.clamp(position, 0.105, 0.27);
>>>>>>> nicky
    actuator.set(v);
    //actuator2.set(v);
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

  public void distanceControl() {
    isDistanceControl = true;
  }

  public void stopDistanceControl() {
    isDistanceControl = false;
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

<<<<<<< HEAD
  public Command constantUpdateCommand() {
    return Commands.runOnce(() -> distanceControl());
  } 

  public Command STOPconstantUpdateCommand() {
    return Commands.runOnce(() -> stopDistanceControl());
  } 
  int j=0;
@Override
  public void periodic() {
    //   if (j==5){
    // // System.out.println(lastCommanded);}
    // j=j+1;
    // j=j%6;
=======
  public Command activateDistanceControl() {
    return Commands.runOnce(() -> distanceControl());
  } 

  public Command stopDistanceControlCommand() {
    return Commands.runOnce(() -> stopDistanceControl());
  } 

@Override
  public void periodic() {
    //System.out.println(lastCommanded);
>>>>>>> nicky

    if (isUppies) {
      lastCommanded += 0.005;
      setPosition(lastCommanded);
    } else if (isDownies) {
      lastCommanded -= 0.005;
      setPosition(lastCommanded);
    }

<<<<<<< HEAD
    Logger.recordOutput("Hood/Position", getPositionCentimeters());

    if (isDistanceControl) {
      final Translation2d blueHopper = new Translation2d(4.623, 4.01);
      double distance = Math.hypot((blueHopper.minus(drive.getPose().getTranslation())).getX(), (blueHopper.minus(drive.getPose().getTranslation())).getY());
      lastCommanded = MathUtil.clamp(distance / 36, -1, 1); 
      setPosition(lastCommanded); // Example scaling factor
    }

=======
    if (isDistanceControl) {
      final Translation2d blueHopper = new Translation2d(4.623, 4.01);
      double distance = Math.hypot((blueHopper.minus(drive.getPose().getTranslation())).getX(), (blueHopper.minus(drive.getPose().getTranslation())).getY());
      //Logger.recordOutput("Hood/DistanceToHopper", distance);
      lastCommanded = distance / 7; 
      setPosition(lastCommanded); // Example scaling factor
    }

    //Logger.recordOutput("Hood/Position", lastCommanded);

>>>>>>> nicky

  }

}
