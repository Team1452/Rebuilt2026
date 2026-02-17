package frc.robot.commands;

import static frc.robot.util.PhoenixUtil.*;

import java.util.function.DoubleSupplier;

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
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;



public class MultiCommands {

    public MultiCommands() {}

    public static Command PushAndShootCommand(Indexer indexer, Shooter shooter) {
        return Commands.parallel(
            shooter.setShooterCommand(0.5),
            indexer.setSpindex(0.35));
    }
    
}
