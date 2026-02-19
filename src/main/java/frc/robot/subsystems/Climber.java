package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;




public class Climber extends SubsystemBase {

	public static final class Config {
		public static final int TALON_CAN_ID = 11; 
		public static final int LIMIT_ANALOG_CHANNEL = 0; 
		public static final double LIMIT_VOLTAGE_THRESHOLD = 2.0;
		public static final double DEFAULT_SPEED = 0.5;

		// Absolute positions (in rotations) for the two mechanical endpoints. Adjust to
		// match your climber geometry. Commonly, retracted = 0.0 and extended = some
		// positive number of rotations.
		public static final double RETRACTED_POSITION_ROTATIONS = 0.0;
		public static final double EXTENDED_POSITION_ROTATIONS = 10.0;
	}

	private final TalonFX motor;
	private final AnalogInput limitSwitch;
	private final TalonFXConfiguration motorConfig = new TalonFXConfiguration();

	private boolean zeroed = false;
	private double lastCommandedSpeed = 0.0;
	private boolean limitPreviouslyClosed = false;
	public boolean fullyExtended = false;
	public boolean fullyRetracted = false;

	public Climber() {
		motor = new TalonFX(Config.TALON_CAN_ID);
		limitSwitch = new AnalogInput(Config.LIMIT_ANALOG_CHANNEL);

		motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

		motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Config.EXTENDED_POSITION_ROTATIONS;
		motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
		motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Config.RETRACTED_POSITION_ROTATIONS;

		motor.getConfigurator().apply(motorConfig, 0.25);
	}

	public void extend(double speed) {
        if (isAtLimit()) {
            stop();
        } else {
            motor.set(speed);
            lastCommandedSpeed = speed;
        }
    }

	/** Stop the climber motor. */
	public void stop() {
		lastCommandedSpeed = 0.0;
		motor.stopMotor();
	}

	public boolean isAtLimit() {
		double v = limitSwitch.getVoltage();
		// Depending on your hardware, triggered may be above or below threshold. Adjust as needed.
		return v >= Config.LIMIT_VOLTAGE_THRESHOLD;
	}

	public double getPosition() {
		return motor.getPosition().getValueAsDouble();
	}
	
	public void zeroPosition() {
		zeroed = true;
		// Also update TalonFX internal position (timeout 0.25s)
		motor.setPosition(0.0, 0.25);
	}

	public boolean isZeroed() {
		return zeroed;
	}

	@Override
	public void periodic() {
		System.out.println(limitSwitch.getVoltage());

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
					double targetAbsolute;
					if (lastCommandedSpeed > 0.0) {
						// Was moving outward / extending -> we hit the EXTENDED endpoint
						targetAbsolute = Config.EXTENDED_POSITION_ROTATIONS;
						fullyExtended = true;
						fullyRetracted = false;
					} else if (lastCommandedSpeed < 0.0) {
						// Was moving inward / retracting -> we hit the RETRACTED endpoint
						targetAbsolute = Config.RETRACTED_POSITION_ROTATIONS;
						fullyExtended = false;
						fullyRetracted = true;
					} else {
						// Unknown direction (motor stationary) — default to RETRACTED
						targetAbsolute = Config.RETRACTED_POSITION_ROTATIONS;
						fullyExtended = false;
						fullyRetracted = true;
					}
					// Set offset so raw + offset == targetAbsolute
					// Update Talon internal position so encoder reading matches the known absolute endpoint.
					// Use a small timeout (0.25s) like zeroPosition() does.
					motor.setPosition(targetAbsolute, 0.25);
					zeroed = true;
				}
				limitPreviouslyClosed = closed;

	}

    public Command extendCommand() {
        return Commands.run(() -> extend(Config.DEFAULT_SPEED), this);
    }

}
