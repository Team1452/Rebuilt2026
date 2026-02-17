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

    private TalonFX spinner;
    private TalonFX spinner2;
    
    public Indexer() {
        spinner = new TalonFX(42, TunerConstants.kCANBus2);
        spinner2 = new TalonFX(61, TunerConstants.kCANBus2);
    }

    public void setIndexer(double velocity) {
        spinner.set(-1 * velocity);
        spinner2.set(velocity);
    }

    public void stopIndexer() {
        spinner.stopMotor();
        spinner2.stopMotor();
    }

    public Command setSpindex(double velocity) {
        return Commands.runOnce(() -> setIndexer(velocity));
    }

     public Command stopCommand() {
        return Commands.runOnce(() -> stopIndexer());
    }


    
}
