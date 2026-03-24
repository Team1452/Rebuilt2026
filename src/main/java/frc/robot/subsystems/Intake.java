package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.generated.TunerConstants;
import static frc.robot.util.PhoenixUtil.*;



public class Intake extends SubsystemBase{

    private TalonFX rotator;
    private TalonFX sucker;
    private TalonFXConfiguration rotatorConfig;
    private TalonFXConfiguration suckerConfig;
    private final double deployRotations = 62.6;
    private final double trenchRotations = 40.0;
    final PositionTorqueCurrentFOC m_request = new PositionTorqueCurrentFOC(0);

    
    public Intake() {
        rotator = new TalonFX(TunerConstants.rotatorMotorID, TunerConstants.kCANBus2);
        sucker = new TalonFX(TunerConstants.suckerMotorID, TunerConstants.kCANBus2);
        rotatorConfig = new TalonFXConfiguration();
        suckerConfig = new TalonFXConfiguration();

        rotatorConfig.Slot0.kP = 5.0;
        suckerConfig.Slot0.kP = 5.0;

        rotatorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		rotatorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 71;

        rotatorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        rotatorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

        rotator.getConfigurator().apply(rotatorConfig, 0.25);
        sucker.getConfigurator().apply(suckerConfig, 0.25);
    }

    public void setSucker(double speed) {
        sucker.set(speed);
    }

    public void stopSucker() {
        sucker.stopMotor();
    }

    public void setRotator(double speed) {
        rotator.set(speed);
    }

    public void zeroPosition() {
        rotator.setPosition(0);
    }

    public void setIntakeAngle(double rotations) {
        rotator.setControl(m_request.withPosition(rotations));
        //Logger.recordOutput("Intake/CommandedAngleRot", rotations);
    }

    public Command setRotatorCommand(double speed) {
        return Commands.runOnce(() -> setRotator(speed), this);
    }

    public Command setAngle(double rotations) {
        return Commands.runOnce(() -> setIntakeAngle(rotations), this);
    }

    public Command setSuckerCommand(double speed) {
        return Commands.runOnce(() -> setSucker(speed), this);
    }

    public Command stopSuckerCommand() {
        return Commands.runOnce(() -> stopSucker(), this);
    }

    public Command deployIntake() {
        return Commands.sequence(
            setAngle(deployRotations).until(() -> rotator.getPosition().getValueAsDouble() > 85),
            setSuckerCommand(0.55)
            );
    }

    public Command retractIntake() {
        return Commands.sequence(
            stopSuckerCommand(),
            setAngle(0)
        );
    }

    public Command zeroCommand() {
        return Commands.runOnce(() -> zeroPosition());
    }

    public Command trenchTime() {
        return Commands.runOnce(
            () -> setAngle(trenchRotations).until(() -> rotator.getPosition().getValueAsDouble() > 85));
    }


    // changes the encoder position, does not actually move it
    public Command setRotatorPosition(double rotations) {
        return Commands.runOnce(() -> rotator.setPosition(rotations), this);
    }

    public Command JIGGLE() {
        return Commands.repeatingSequence(
            setAngle(40),
            Commands.waitSeconds(0.5),
            setAngle(35),
            Commands.waitSeconds(0.5)
        );
    }


    
}
