package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;


public class Climber extends SubsystemBase {

	/** Simple config placeholders — replace with real IDs. */
	public static final class Config {
		public static final int TALON_CAN_ID = 11; // TODO: change to real CAN ID
		public static final int LIMIT_ANALOG_CHANNEL = 0; // TODO: change to real analog channel
		public static final double LIMIT_VOLTAGE_THRESHOLD = 2.0; // voltage above/below which the limit is considered pressed
		public static final double DEFAULT_SPEED = 0.5;

		// Absolute positions (in rotations) for the two mechanical endpoints. Adjust to
		// match your climber geometry. Commonly, retracted = 0.0 and extended = some
		// positive number of rotations.
		public static final double RETRACTED_POSITION_ROTATIONS = 0.0;
		public static final double EXTENDED_POSITION_ROTATIONS = 10.0; // TODO: set to real value
	}

	private final TalonFX motor;
	private final AnalogInput limitSwitch;

	private boolean zeroed = false;

	private double lastCommandedSpeed = 0.0;

	private boolean limitPreviouslyClosed = false;

	/**
	 * Create a Climber subsystem using explicit hardware IDs.
	 *
	 * @param talonCanId CAN ID for the TalonFX
	 * @param limitAnalogChannel analog channel for the limit switch
	 */
	public Climber(int talonCanId, int limitAnalogChannel) {
		motor = new TalonFX(talonCanId);
		limitSwitch = new AnalogInput(limitAnalogChannel);
	}

	/** Convenience constructor that uses values from Config. */
	public Climber() {
		this(Config.TALON_CAN_ID, Config.LIMIT_ANALOG_CHANNEL);
	}

	public void extend(double speed) {
        if (isAtLimit()) {
            // At limit and trying to extend further -> stop
            stop();
        } else {
            // Otherwise, set the motor speed as commanded
            motor.set(speed);
            lastCommandedSpeed = speed;
        }
    }

	/** Stop the climber motor. */
	public void stop() {
		lastCommandedSpeed = 0.0;
		motor.stopMotor();
	}

	/** Return true when the analog limit switch is triggered. */
	public boolean isAtLimit() {
		double v = limitSwitch.getVoltage();
		// Depending on your hardware, triggered may be above or below threshold. Adjust as needed.
		return v >= Config.LIMIT_VOLTAGE_THRESHOLD;
	}

	public double getPosition() {
		return motor.getPosition().getValueAsDouble();
	}

	
	public void zeroPosition() {
		double raw = motor.getPosition().getValueAsDouble();
		zeroed = true;
		// Also update TalonFX internal position (timeout 0.25s)
		motor.setPosition(0.0, 0.25);
	}

	public boolean isZeroed() {
		return zeroed;
	}

	private double clampSpeed(double speed) {
		if (speed > 1.0) return 1.0;
		if (speed < -1.0) return -1.0;
		return speed;
	}

	@Override
	public void periodic() {
		// Put subsystem periodic code here. E.g. telemetry for tuning/debug.
		SmartDashboard.putBoolean("Climber/AtLimit", isAtLimit());
		SmartDashboard.putNumber("Climber/LimitVoltage", limitSwitch.getVoltage());
		SmartDashboard.putNumber("Climber/Position", getPosition());
		SmartDashboard.putBoolean("Climber/Zeroed", isZeroed());

				// Perform a one-shot calibration when the analog loop closes. The loop
				// can be closed at either mechanical extreme; we use the last commanded
				// motor direction to decide whether this corresponds to the extended or
				// retracted endpoint.
				boolean closed = isAtLimit();
				if (closed && !limitPreviouslyClosed) {
					// Rising edge: loop just closed
					double raw = motor.getPosition().getValueAsDouble();
					double targetAbsolute;
					if (lastCommandedSpeed > 0.0) {
						// Was moving outward / extending -> we hit the EXTENDED endpoint
						targetAbsolute = Config.EXTENDED_POSITION_ROTATIONS;
					} else if (lastCommandedSpeed < 0.0) {
						// Was moving inward / retracting -> we hit the RETRACTED endpoint
						targetAbsolute = Config.RETRACTED_POSITION_ROTATIONS;
					} else {
						// Unknown direction (motor stationary) — default to RETRACTED
						targetAbsolute = Config.RETRACTED_POSITION_ROTATIONS;
					}
					// Set offset so raw + offset == targetAbsolute
					zeroed = true;
				}
				limitPreviouslyClosed = closed;

	}

    public Command extendCommand() {
        return Commands.run(() -> extend(Config.DEFAULT_SPEED), this);
    }

}
