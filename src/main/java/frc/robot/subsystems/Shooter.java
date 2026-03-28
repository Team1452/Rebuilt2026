package frc.robot.subsystems;

import static frc.robot.util.PhoenixUtil.*;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
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
import com.ctre.phoenix6.signals.NeutralModeValue;
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
    SlewRateLimiter filter = new SlewRateLimiter(10);
    private double power = 0.0;
    private double rampPower = 0;
    
    public Shooter() {
        gunWheel = new TalonFX(TunerConstants.gunWheelMotorID,  TunerConstants.kCANBus2);
        follower = new TalonFX(TunerConstants.gunFollowerMotorID, TunerConstants.kCANBus2);

        gunConfig = new TalonFXConfiguration();
        followerConfig = new TalonFXConfiguration();

        gunConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        gunConfig.Slot0 = gunGains;
        followerConfig.Slot0 = gunGains;

        gunConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        followerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        gunWheel.getConfigurator().apply(gunConfig, 0.25);
        follower.getConfigurator().apply(followerConfig, 0.25);

        filter.reset(0);

        follower.setControl(new Follower(gunWheel.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    public void setShooterFractional(double velocity) {
        gunWheel.set(velocity);
    }

    public void setShooterVelocity(double rps) {
        gunWheel.setControl(new VelocityVoltage(rps));
    }

    public void stopShooter() {
        //filter.reset(0);
        rampPower = 0;
        gunWheel.stopMotor();
    }

    public void incrementPower(double increment) {
        rampPower = MathUtil.clamp(rampPower + increment, 0, 300);
    }

    public Command incrementPowerCommand(double increment) {
        return Commands.runOnce(() -> incrementPower(increment));
    }

    public Command shootPowerCommand() {
        return Commands.runOnce(() -> setShooterFractional(power), this);
    }

    public Command simpleShoot() {
        return Commands.sequence(
            Commands.runOnce(() -> setShooterFractional(0.1)), 
            Commands.waitSeconds(2), 
            Commands.runOnce(() ->  setShooterFractional(0)));
    }

    public Command IBegTheeStop() {
        return Commands.runOnce(() -> stopShooter());
    }

    public Command controllerShoot(double rps) {
        return Commands.sequence(
            Commands.runOnce(() -> setShooterVelocity(rps)), 
            Commands.waitSeconds(2), 
            Commands.runOnce(() -> setShooterVelocity(0)));
    }

    // shooting using fractional power
    public Command setShooterFractionalCommand(double fractional) {
        return Commands.runOnce(() -> setShooterFractional(fractional));
    }

    // shooting using velocity control
    public Command setShooterCommand(double rps) {
        return Commands.runOnce(() -> setShooterVelocity(rps));
    }

    public void getSettings(Drive drive) {
        ShooterInterpolation interpolation = new ShooterInterpolation();
        double[] settings = interpolation.getSettings(drive);
        double power = settings[0];
        setShooterFractional(power);
        Logger.recordOutput("Shooter/InterpolatedPower", power);
        //Logger.recordOutput("Shooter/InterpolatedAngle", settings[1]);
    }

    // shooting using interpolation
    public Command interpolatedShootingCommand(Drive drive) {
        return Commands.run(() -> getSettings(drive));
    }

    public void getPower(Drive drive) {
        double distance = (DriveCommands.findDistance(drive).getAsDouble() * 39.3701) - 37;
        double power = 12.133 * Math.pow(distance, 0.34);
        setShooterFractional(power);
    }

    // shooting using distance calculation
    public Command distanceShootingCommand(Drive drive) {
        return Commands.run(() -> getPower(drive));
    }

    public Command rampUpCommand(double rps){
        return Commands.run(() -> setShooterVelocity(filter.calculate(rps)));
    }

    public void setRampPower(double rp) {
        rampPower = rp;
    }

    public Command setRampPowerCommand(double rp) {
        return Commands.runOnce(() -> setRampPower(rp));
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Shooter/RampPower-Strength", rampPower);
        Logger.recordOutput("Shooter/Current-RPS", gunWheel.getVelocity().getValueAsDouble());
        //Logger.recordOutput("Shooter/GunWheel Velocity", gunWheel.getVelocity().getValueAsDouble());
        //Logger.recordOutput("Shooter/Follower Velocity", follower.getVelocity().getValueAsDouble());

        setShooterVelocity(filter.calculate(rampPower));
    }
    

}


