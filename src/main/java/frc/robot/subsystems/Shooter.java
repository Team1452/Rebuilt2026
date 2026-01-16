package frc.robot.subsystems;

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

    private TalonFX gunWheel;
    private TalonFXConfiguration gunConfig;
    private static final Slot0Configs gunGains = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0.5)
        .withKS(0.1).withKV(2.66).withKA(0)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    
    public Shooter() {
        gunWheel = new TalonFX(37, "");
        gunConfig.Slot0 = gunGains;

    }

    public void setShooter(double velocity) {
        gunWheel.set(velocity);
    }

    public void setShooter2(double rpm) {
        gunWheel.setControl(new VelocityVoltage(rpm));
    }

    @Override
    public void periodic() {
    }

    public Command simpleShoot() {
        return Commands.sequence(
            Commands.run(() -> setShooter(1)), 
            new WaitCommand(2), 
            Commands.run(() -> setShooter(0)));
    }

}
