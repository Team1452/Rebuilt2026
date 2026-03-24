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

import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.ShooterInterpolation;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import java.util.TreeMap;


public class Shooter extends SubsystemBase{

    private TalonFX gunWheel;
    private TalonFX follower;
    private TalonFXConfiguration gunConfig;
    private TalonFXConfiguration followerConfig;
    private static final Slot0Configs gunGains = new Slot0Configs()
        .withKP(0.4).withKI(0).withKD(0)
        .withKS(0.1).withKV(0.125).withKA(0).
        withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    private double power = 0.0;
    
    public Shooter() {
        gunWheel = new TalonFX(TunerConstants.gunWheelMotorID,  TunerConstants.kCANBus2);
        follower = new TalonFX(TunerConstants.gunFollowerMotorID, TunerConstants.kCANBus2);

        gunConfig = new TalonFXConfiguration();
        followerConfig = new TalonFXConfiguration();

        gunConfig.Slot0 = gunGains;
        followerConfig.Slot0 = gunGains;

        gunConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        followerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
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
        gunWheel.set(0);
        gunWheel.stopMotor();
    }

    public void incrementPower(double increment) {
        power = MathUtil.clamp(power + increment, 0, 300);
        setShooter2(power);
    }

    public Command incrementPowerCommand(double increment) {
        return Commands.runOnce(() -> incrementPower(increment));
    }

    public Command shootPowerCommand() {
        return Commands.runOnce(() -> setShooter(power), this);
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

    public Command setShooterCommand(double fractional) {
        return Commands.runOnce(() -> setShooter(fractional));
    }

    public Command setShooterCommand2(double rps) {
        return Commands.runOnce(() -> setShooter2(rps));
    }

    public void getSettings(Drive drive) {
        ShooterInterpolation interpolation = new ShooterInterpolation();
        double[] settings = interpolation.getSettings(drive);
        double power = settings[0];
        setShooter2(power);
        Logger.recordOutput("Shooter/InterpolatedPower", power);
        //Logger.recordOutput("Shooter/InterpolatedAngle", settings[1]);
    }

    public Command setShooterCommand3(Drive drive) {
        return Commands.run(() -> getSettings(drive));
    }

    public void getPowerLaw(Drive drive) {
        double distance = (DriveCommands.findDistance(drive).getAsDouble() * 39.3701) - 37;
        double power = 12.133 * Math.pow(distance, 0.34);
        setShooter2(power);
        Logger.recordOutput("Shooter/PowerLawEquation", power);
    }

    public Command setShooterCommand4(Drive drive) {
        return Commands.run(() -> getPowerLaw(drive));
    }

    public void getPowerLinear(Drive drive) {
        double distance = (DriveCommands.findDistance(drive).getAsDouble() * 39.3701) - 37;
        double power = (0.2309 * distance) + 34.123;
        setShooter2(power);
        Logger.recordOutput("Shooter/PowerLawEquation", power);
    }

    public Command setShooterCommand5(Drive drive) {
        return Commands.run(() -> getPowerLinear(drive));
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Shooter/PowerShot-Strength", power);
        //Logger.recordOutput("Shooter/GunWheel Velocity", gunWheel.getVelocity().getValueAsDouble());
        //Logger.recordOutput("Shooter/Follower Velocity", follower.getVelocity().getValueAsDouble());
    }
    

}


