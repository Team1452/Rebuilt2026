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
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import frc.robot.generated.TunerConstants;

public class Indexer extends SubsystemBase{    

    private TalonFX tornado;
    private TalonFX dustdevil;
    private TalonFX hotdog;

    private TalonFXConfiguration tornadoConfig;
    private TalonFXConfiguration dustdevilConfig;
    private TalonFXConfiguration hotdogConfig;

    private static final Slot0Configs tornadoGains = new Slot0Configs()
        .withKP(0.1).withKI(0).withKD(0.5)
        .withKS(0.01).withKV(2).withKA(0).
        withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    private static final Slot0Configs dogGains = new Slot0Configs().
        withKP(0.1).withKI(0).withKD(0.5)
        .withKS(0.01).withKV(2).withKA(0).
        withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    
    public Indexer() {
        tornado = new TalonFX(TunerConstants.tornadoMotorID, TunerConstants.kCANBus2);
        dustdevil = new TalonFX(TunerConstants.dustdevilMotorID, TunerConstants.kCANBus2);
        hotdog = new TalonFX(TunerConstants.hotdogMotorID, TunerConstants.kCANBus2);

        tornadoConfig = new TalonFXConfiguration();
        dustdevilConfig = new TalonFXConfiguration();
        hotdogConfig = new TalonFXConfiguration();

        tornadoConfig.Slot0 = tornadoGains;
        dustdevilConfig.Slot0 = tornadoGains;
        hotdogConfig.Slot0 = dogGains;

        tornadoConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        dustdevilConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        hotdogConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        tornado.getConfigurator().apply(tornadoConfig, 0.25);
        dustdevil.getConfigurator().apply(dustdevilConfig, 0.25);
        hotdog.getConfigurator().apply(hotdogConfig, 0.25);

        dustdevil.setControl(new Follower(tornado.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    public void setSpindex(double velocity) {
        tornado.set(velocity);
    }

    public void setHotdog(double velocity) {
        hotdog.set(velocity);
    }

    public void stopIndexer() {
        tornado.stopMotor();
        hotdog.stopMotor();
    }

    public Command setSpindexCommand(double velocity) {
        return Commands.runOnce(() -> setSpindex(velocity));
    }

    public Command setHotdogCommand(double velocity) {
        return Commands.runOnce(() -> setHotdog(velocity));
    }

    public Command activatePorknado(double velocity, double hotdogVelocity) {
        return Commands.parallel(
            Commands.runOnce(() -> setSpindex(velocity)),
            Commands.runOnce(() -> setHotdog(hotdogVelocity))
        ); 
    }

     public Command stopCommand() {
        return Commands.runOnce(() -> stopIndexer());
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Tornado Velocity", tornado.getVelocity().getValueAsDouble());
        Logger.recordOutput("DustDevil Velocity", dustdevil.getVelocity().getValueAsDouble());
        Logger.recordOutput("Hotdog Velocity", hotdog.getVelocity().getValueAsDouble());
    }


    
}
