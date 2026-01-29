package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import static frc.robot.util.PhoenixUtil.*;



public class Intake extends SubsystemBase{

    private TalonFX rotator;
    private TalonFX sucker;
    private TalonFXConfiguration rotatorConfig;
    private TalonFXConfiguration suckerConfig;
    final PositionVoltage m_request = new PositionVoltage(0);

    
    public Intake() {
        rotator = new TalonFX(0);
        sucker = new TalonFX(1);
        rotatorConfig = new TalonFXConfiguration();
        suckerConfig = new TalonFXConfiguration();

        rotatorConfig.Slot0.kP = 5.0;
        suckerConfig.Slot0.kP = 5.0;

        tryUntilOk(5, () -> rotator.getConfigurator().apply(rotatorConfig, 0.25));
        tryUntilOk(5, () -> sucker.getConfigurator().apply(suckerConfig, 0.25));
    }

    public void setSucker(double speed) {
        sucker.set(speed);
    }

    public void stopSucker() {
        sucker.stopMotor();
    }

    public void setIntakeAngle(double degrees) {
        final double MIN_DEGREES = -180.0;
        final double MAX_DEGREES = 180.0;
        double clampedDegrees = MathUtil.clamp(degrees, MIN_DEGREES, MAX_DEGREES);
        double clampedRotations = clampedDegrees / 360.0;

        rotator.setControl(m_request.withPosition(clampedRotations));

        Logger.recordOutput("Intake/CommandedAngleDeg", clampedDegrees);
    }

    public Command setAngle(double degrees) {
        return Commands.runOnce(() -> setIntakeAngle(degrees), this);
    }

    public Command setSuckerCommand(double speed) {
        return Commands.runOnce(() -> setSucker(speed), this);
    }

    public Command stopSuckerCommand() {
        return Commands.runOnce(() -> stopSucker(), this);
    }






    
}
