package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
    private final double deployRotations = 63.8;
    private final double trenchRotations = 30.0;
    // final PositionTorqueCurrentFOC m_request = new PositionTorqueCurrentFOC(0);
    // private final com.ctre.phoenix6.controls.MotionMagicVoltage m_request = new com.ctre.phoenix6.controls.MotionMagicVoltage(0);
    private boolean isDeployed = false;
    private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    public double suckerCurrent = 0.0;

    
    public Intake() {
        rotator = new TalonFX(TunerConstants.rotatorMotorID, TunerConstants.kCANBus2);
        sucker = new TalonFX(TunerConstants.suckerMotorID, TunerConstants.kCANBus2);
        rotatorConfig = new TalonFXConfiguration();
        suckerConfig = new TalonFXConfiguration();

        // SLOT 0: Standard Movement (Fast)
        rotatorConfig.Slot0.kP = 12.0; // Higher P for 125:1 ratio
        rotatorConfig.Slot0.kV = 0.12;
        rotatorConfig.MotionMagic.MotionMagicCruiseVelocity = 120; 

        // SLOT 1: Jiggle Movement (Slow)
        rotatorConfig.Slot1.kP = 15.0;
        rotatorConfig.Slot1.kV = 0.12;

        rotatorConfig.MotionMagic.MotionMagicCruiseVelocity = 60; // Slow speed for jiggle
        rotatorConfig.MotionMagic.MotionMagicAcceleration = 100;

        rotatorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		rotatorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 71;

        rotatorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        rotatorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

        rotator.getConfigurator().apply(rotatorConfig, 0.25);
        sucker.getConfigurator().apply(suckerConfig, 0.25);

        rotator.getConfigurator().apply(rotatorConfig);
        
    }

    public void setSucker(double speed) {
        sucker.set(speed * -1);
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

    // public void setIntakeAngle(double rotations) {
    //     rotator.setControl(m_request.withPosition(rotations));
    //     //Logger.recordOutput("Intake/CommandedAngleRot", rotations);
    // }
    public void setIntakeAngle(double rotations, int slot) {
        rotator.setControl(m_request.withPosition(rotations).withSlot(slot));
    }


//     public void setIntakeAngle(double rotations, double velocity) {
//     // Update the speed constraint dynamically if needed
//     System.out.println("Moving to: " + rotations + " at speed: " + velocity);
//     var mConfigs = new com.ctre.phoenix6.configs.MotionMagicConfigs();
//     mConfigs.MotionMagicCruiseVelocity = velocity;
//     rotator.getConfigurator().apply(mConfigs);
//     rotator.setControl(m_request.withPosition(rotations));
// }

    public Command setRotatorCommand(double speed) {
        return Commands.runOnce(() -> setRotator(speed), this);
    }

    public Command setAngle(double rotations) {
        return Commands.runOnce(() -> setIntakeAngle(rotations, 0), this); // Uses Slot 0 (Fast)
    }

    public Command setSuckerCommand(double speed) {
        return Commands.runOnce(() -> setSucker(speed), this);
    }

    public Command stopSuckerCommand() {
        return Commands.runOnce(() -> stopSucker(), this);
    }

    public Command deployIntake() {
        isDeployed = true;
        return Commands.sequence(
            setAngle(deployRotations),
            Commands.waitSeconds(1),
            setSuckerCommand(0.75)
            );
    }

    public Command retractIntake() {
        isDeployed = false;
        return Commands.sequence(
            stopSuckerCommand(),
            setAngle(0)
        );
    }

    public Command intaking() {
        if (isDeployed == false) {
            return deployIntake();
        } else {
            return retractIntake();
        }
    }

    public Command trenchMode() {
        if (rotator.getPosition().getValueAsDouble() < 31) {
            return Commands.sequence(setAngle(trenchRotations)); 
        } else {
            return Commands.none();
        }
    }

    public Command zeroCommand() {
        return Commands.runOnce(() -> zeroPosition());
    }

    // changes the encoder position, does not actually move it
    public Command setRotatorPosition(double rotations) {
        return Commands.runOnce(() -> rotator.setPosition(rotations), this);
    }

    public Command JIGGLE() {
        return Commands.repeatingSequence(
            Commands.runOnce(() -> setIntakeAngle(40, 1), this), // Uses Slot 1 (Slow)
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> setIntakeAngle(20, 1), this), // Uses Slot 1 (Slow)
            Commands.waitSeconds(0.5)
    );
    }
    public Command unclogCommand(){
        return Commands.sequence(
            setSuckerCommand(-0.2),
            Commands.waitSeconds(0.5),
            setSuckerCommand(0.75)
        );
    }
    
    @Override
    public void periodic() {
        suckerCurrent = sucker.getSupplyCurrent().getValueAsDouble();
        Logger.recordOutput("Intake/IntakeCurrent", suckerCurrent);
        if (suckerCurrent > 80) {
            System.out.println("Intake stalled! Current: " + suckerCurrent);
            unclogCommand();
        } 
    }
    
}
