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
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;




public class MultiCommands {

    public MultiCommands() {}

    public static Command autoIntakeCommand(Intake intake, double waitTime) {
        return Commands.sequence(
            intake.deployIntake(),
            Commands.waitSeconds(waitTime),
            intake.stopSuckerCommand()
        );
    }

    public static Command autoShootCommand(Indexer indexer, Shooter shooter, double waitTime, double rps) {
        return Commands.sequence(
            shooter.setRampPowerCommand(rps),
            Commands.waitSeconds(1),
            indexer.activatePorknado(-0.4, 0.3), 
            Commands.waitSeconds(waitTime),
            Commands.parallel(
                indexer.activatePorknado(0, 0),
                shooter.IBegTheeStop())
            );
    }

    public static Command ShootAndJiggle(Intake intake, Indexer indexer) {
        return Commands.sequence(
            indexer.activatePorknado(-0.4, 0.7),
            Commands.waitSeconds(2),
            intake.JIGGLE()
        );

    }
    
    public static Command goShootPosition(Shooter shooter, Indexer indexer, Hood hood) {
        return Commands.sequence(
            Commands.parallel(
               shooter.setShooterCommand(2.75),
                hood.setPositionCommand(-0.02)
        ), 
        Commands.waitSeconds(0.5),
        indexer.activatePorknado(-0.4, 0.5));
    }

    public static Command stopping(Shooter shooter, Indexer indexer, Intake intake) {
        return
            Commands.parallel(
                indexer.activatePorknado(0, 0),
                shooter.setShooterCommand(0.0),
                intake.setSuckerCommand(0)
                ); 
    }

    public static Command goingUnder(Intake intake, Hood hood) {
        return Commands.parallel(
            intake.trenchMode(),
            hood.goFlat()
        );
    } 

}
