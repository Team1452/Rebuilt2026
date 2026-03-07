package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import static edu.wpi.first.units.Units.Amps;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.*;



public class Climber extends SubsystemBase {

	public static final class Config {
		public static final double DEFAULT_SPEED = 0.5;

		public static final double RETRACTED_POSITION_ROTATIONS = 0.0;
		public static final double EXTENDED_POSITION_ROTATIONS = 10.0;
	}

	private final TalonFX motor;
	private final TalonFX follower;
	private final DigitalInput limitSwitch1;
	private final DigitalInput limitSwitch2;
	private final TalonFXConfiguration motorConfig = new TalonFXConfiguration();

	private boolean zeroed = false;
	private double lastCommandedSpeed = 0.0;
	private boolean limitPreviouslyClosed = false;
	public boolean fullyExtended = false;
	public boolean fullyRetracted = false;

	public Climber() {
		motor = new TalonFX(TunerConstants.climberMotorID, TunerConstants.kCANBus2);
		follower = new TalonFX(TunerConstants.climberFollowerMotorID, TunerConstants.kCANBus2);
		limitSwitch1 = new DigitalInput(0);
		limitSwitch2 = new DigitalInput(1);

		motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		motorConfig.withCurrentLimits(
            new CurrentLimitsConfigs()
                // Swerve azimuth does not require much torque output, so we can set a relatively low
                // stator current limit to help avoid brownouts without impacting performance.
                .withStatorCurrentLimit(Amps.of(20))
                .withStatorCurrentLimitEnable(true)
        );

		motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Config.EXTENDED_POSITION_ROTATIONS;
		motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
		motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Config.RETRACTED_POSITION_ROTATIONS;

		motor.getConfigurator().apply(motorConfig, 0.25);
		follower.getConfigurator().apply(motorConfig, 0.25);

		follower.setControl(new Follower(motor.getDeviceID(), MotorAlignmentValue.Aligned));
	}

	public void move(double speed) {
        if (isAtLimit()) {
			if (lastCommandedSpeed > 0 && speed > 0) {
				stop();
			} else if (lastCommandedSpeed > 0 && speed < 0) {
				motor.set(speed);
				lastCommandedSpeed = speed;
			} else if (lastCommandedSpeed < 0 && speed < 0) {
				stop();
			} else if (lastCommandedSpeed > 0 && speed < 0) {
				motor.set(speed);
				lastCommandedSpeed = speed;
			}
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
		// Depending on your hardware, triggered may be above or below threshold. Adjust as needed.
		if (limitSwitch1.get() || limitSwitch2.get()) {
			return true;
		} else {
			return false;
		}
	}

	public double getPosition() {
		return motor.getPosition().getValueAsDouble();
	}
	
	public void zeroPosition() {
		motor.setPosition(0);
		follower.setPosition(0);
	}

	@Override
	public void periodic() {
		// Put subsystem periodic code here. E.g. telemetry for tuning/debug.
		//Logger.recordOutput("Climber/AtLimit", isAtLimit());
		//Logger.recordOutput("Climber/LimitVoltage", limitSwitch.getVoltage());
		//Logger.recordOutput("Climber/Position", getPosition());
		//Logger.recordOutput("Climber/Zeroed", isZeroed());

				// Perform a one-shot calibration when the analog loop closes. The loop
				// can be closed at either mechanical extreme; we use the last commanded
				// motor direction to decide whether this corresponds to the extended or
				// retracted endpoint.
				/* boolean closed = isAtLimit();
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
					} */
					// Set offset so raw + offset == targetAbsolute
					// Update Talon internal position so encoder reading matches the known absolute endpoint.
					// Use a small timeout (0.25s) like zeroPosition() does.
					//motor.setPosition(targetAbsolute, 0.25);
					//zeroed = true;
				//}
				//limitPreviouslyClosed = closed;

	}

    public Command extendCommand() {
        return Commands.runOnce(() -> move(Config.DEFAULT_SPEED)).until(() -> isAtLimit());
    }

	public Command retractCommand() {
		return Commands.runOnce(() -> move(-1 * Config.DEFAULT_SPEED)).until(() -> isAtLimit());
	}

	public Command zeroCommand() {
		return Commands.runOnce(() -> zeroPosition());
	}

	

}
