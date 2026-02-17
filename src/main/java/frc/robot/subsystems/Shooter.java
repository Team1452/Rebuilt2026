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
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.generated.TunerConstants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class Shooter extends SubsystemBase{    

    private TalonFX gunWheel;
    private TalonFX follower;
    private TalonFXConfiguration gunConfig;
    private TalonFXConfiguration followerConfig;
    private static final Slot0Configs gunGains = new Slot0Configs()
        .withKP(0.1).withKI(0).withKD(0.5)
        .withKS(0.01).withKV(0.1).withKA(0).
        withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    
    public Shooter() {
        gunWheel = new TalonFX(6,  TunerConstants.kCANBus2);
        follower = new TalonFX(7, TunerConstants.kCANBus2);
        gunConfig = new TalonFXConfiguration();
        followerConfig = new TalonFXConfiguration();
        gunConfig.Slot0 = gunGains;
        followerConfig.Slot0 = gunGains;
        gunConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        followerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        gunWheel.getConfigurator().apply(gunConfig, 0.25);
        follower.getConfigurator().apply(followerConfig, 0.25);

        follower.setControl(new Follower(gunWheel.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    public void setShooter(double velocity) {
        gunWheel.set(velocity);
    }

    public void setShooter2(double rps) {
        gunWheel.setControl(new VelocityVoltage(rps));
    }

    public void stopShooter() {
        gunWheel.stopMotor();
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

    public Command controllerShoot(double rps) {
        return Commands.sequence(
            Commands.runOnce(() -> setShooter2(rps)), 
            Commands.waitSeconds(2), 
            Commands.runOnce(() -> setShooter2(0)));
    }

    public Command setShooterCommand(double rps) {
        return Commands.runOnce(() -> setShooter2(rps));
    }
    

}
