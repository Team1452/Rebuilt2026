package frc.robot.subsystems;

import static frc.robot.util.PhoenixUtil.*;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class Shooter extends SubsystemBase{    

    private TalonFX gunWheel1;
    private TalonFX gunWheel2;
    private TalonFXConfiguration gunConfig;
    private static final Slot0Configs gunGains = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0.5)
        .withKS(0.1).withKV(2.66).withKA(0)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    
    public Shooter() {
        gunWheel1 = new TalonFX(6, "");
        gunWheel2 = new TalonFX(7, "");
        gunConfig = new TalonFXConfiguration();
        gunConfig.Slot0 = gunGains;
        tryUntilOk(5, () -> gunWheel1.getConfigurator().apply(gunConfig, 0.25));
        tryUntilOk(5, () -> gunWheel2.getConfigurator().apply(gunConfig, 0.25));
    }

    public void setShooter(double velocity) {
        gunWheel1.set(velocity);
        gunWheel2.set(velocity);
    }

    public void setShooter2(double rpm) {
        gunWheel1.setControl(new VelocityVoltage(rpm));
        gunWheel2.setControl(new VelocityVoltage(rpm));
    }

    public void stopShooter() {
        gunWheel1.stopMotor();
        gunWheel2.stopMotor();
    }

    @Override
    public void periodic() {
    }

    public Command simpleShoot() {
        return Commands.sequence(
            Commands.runOnce(() -> setShooter(0.1)), 
            Commands.waitSeconds(2), 
            Commands.runOnce(() -> setShooter(0)));
    }

    public Command IBegTheeStop() {
        return Commands.runOnce(() -> stopShooter());
    }

    public Command controllerShoot(double rpm) {
        return Commands.sequence(
            Commands.runOnce(() -> setShooter2(rpm)), 
            Commands.waitSeconds(2), 
            Commands.runOnce(() -> setShooter2(0)));
    }

    public Command setShooterCommand(double fractional) {
        return Commands.runOnce(() -> setShooter(fractional));
    }
    

}
