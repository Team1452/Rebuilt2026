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

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import java.util.TreeMap;


public class ShooterInterpolation {
    // A TreeMap stores data in order of the key (Distance)
    private final TreeMap<Double, double[]> shotMap = new TreeMap<>();

    public ShooterInterpolation() {
        // Distance (Inches), {Power, Angle}
        shotMap.put(40.0,  new double[]{48, -0.365});
        shotMap.put(50.0, new double[]{50, -0.365});
        shotMap.put(60.0, new double[]{52, -0.365});
        shotMap.put(70.0, new double[]{54, -0.3655});
        shotMap.put(80.0, new double[]{57, -0.15});
        shotMap.put(90.0, new double[]{60, -0.15});
        shotMap.put(100.0, new double[]{63, 0});
        shotMap.put(110.0, new double[]{66, 0});
        shotMap.put(120.0, new double[]{68, 0.065});
        shotMap.put(130.0, new double[]{71, 0.065});
        shotMap.put(140.0, new double[]{75, 0.065});

    }

    public double[] getSettings(Drive drive) {
        double distance = (DriveCommands.findDistance(drive).getAsDouble() * 39.3701) - 37;
        // 1. Check if distance is exactly in the map or out of bounds
        if (shotMap.containsKey(distance)) return shotMap.get(distance);
        if (distance < shotMap.firstKey()) return shotMap.firstEntry().getValue();
        if (distance > shotMap.lastKey()) return shotMap.lastEntry().getValue();

        // 2. Get the points immediately below and above our current distance
        Double lowKey = shotMap.floorKey(distance);
        Double highKey = shotMap.ceilingKey(distance);

        double[] lowVal = shotMap.get(lowKey);
        double[] highVal = shotMap.get(highKey);

        // 3. The Interpolation Math
        double t = (distance - lowKey) / (highKey - lowKey);
        
        double interpolatedPower = lowVal[0] + t * (highVal[0] - lowVal[0]);
        double interpolatedAngle = lowVal[1] + t * (highVal[1] - lowVal[1]);

        Logger.recordOutput("Shooter/DistanceFromHopper", distance);

        return new double[]{interpolatedPower, interpolatedAngle};
    }
}