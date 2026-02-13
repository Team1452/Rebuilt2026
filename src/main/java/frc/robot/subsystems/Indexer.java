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
import frc.robot.generated.TunerConstants;

public class Indexer extends SubsystemBase{    

    private TalonFX rollerWheel;
    private TalonFX kickerWheel;
    
    public Indexer() {
        rollerWheel = new TalonFX(50, TunerConstants.kCANBus2);
        kickerWheel = new TalonFX(51, TunerConstants.kCANBus2);
    }

    public void setIndexer(double velocity) {
        rollerWheel.set(velocity);
    }

    public void setKicker(double velocity) {
        kickerWheel.set(velocity);
    }

    public void stopIndexer() {
        rollerWheel.stopMotor();
        kickerWheel.stopMotor();
    }

    @Override
    public void periodic() {
    }

    public Command setRollerCommand(double velocity) {
        return Commands.runOnce(() -> setIndexer(velocity));
    }

    public Command setKickerCommand(double velocity) {
        return Commands.runOnce(() -> setKicker(velocity));
    }

     public Command stopCommand() {
        return Commands.runOnce(() -> stopIndexer());
    }


    
}
