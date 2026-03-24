// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.LED.AnimationType;
import frc.robot.subsystems.drive.Drive;

import java.nio.file.Path;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import frc.robot.subsystems.LED.LEDSubsystem;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 5.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2
  final Translation2d blueHopper = new Translation2d(4.6228, 4.01); //hopper point
  private static boolean previousZone = false;

  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY*2, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }

  public static Command centerOnHopperCommand(Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier){
      final Translation2d blueHopper = new Translation2d(4.6228, 4.01);
    // returns x supplier, y supplier and angle supplier that updates

     return joystickDriveAtAngle(drive, xSupplier, ySupplier, () -> blueHopper.minus(drive.getPose().getTranslation()).getAngle());

  }

  public static BooleanSupplier isFacingHopper(Drive drive, double toleranceMeters){
    final Translation2d blueHopper = new Translation2d(4.6228, 4.01);
    return () -> Math.abs(blueHopper.minus(drive.getPose().getTranslation()).getAngle().getDegrees()) < toleranceMeters;
  }
  //finds distance
  public static DoubleSupplier findDistance(Drive drive){
      final Translation2d blueHopper = new Translation2d(4.6228, 4.01);
    return () -> (blueHopper.getDistance(drive.getPose().getTranslation()));
  }
  //finds if in between zone
  public static boolean isShootingZone(Drive drive, Translation2d blueHopper, double minDistance, double maxDistance){
  

  
    double distance = blueHopper.getDistance(drive.getPose().getTranslation());
    return distance >= minDistance && distance <= maxDistance;
  
}
  
  public static Command turnGreen(Drive drive, LEDSubsystem ledSystem){
  
  return Commands.run(() -> {
    boolean inZone = isShootingZone(drive, new Translation2d(4.6228, 4.01), 0.933, 3.233);

    // Only update the animation when the zone changes to avoid
    // constantly sending commands to the LED subsystem when the robot is near the boundary
    if (inZone != previousZone) {
      if (inZone) {
        ledSystem.setAnimation(AnimationType.Rainbow, 1);
        System.out.println("in zone");
      } else {
        ledSystem.setAnimation(AnimationType.ColorFlow, 1);
         System.out.println(" not in zone");
      }
      previousZone = inZone;
    }

  }, ledSystem)
  .beforeStarting(() -> {
    // make sure correct initial animation when the command is first scheduled
    boolean inZone = isShootingZone(drive, new Translation2d(4.6228, 4.01), 3, 3.233);
    if (inZone) {
      ledSystem.setAnimation(AnimationType.Blue, 1);
    } else {
      ledSystem.setAnimation(AnimationType.Red, 1);
    }
    previousZone = inZone;
  });
}

// 1. Define this at the class level
private static String previousState = ""; 

public static Command shootingZones(Drive drive, LEDSubsystem ledSystem) {
    Translation2d targetPos = new Translation2d(4.6228, 4.01);

    return Commands.run(() -> {
        // 2. Logic to determine current state
        boolean inInner = isShootingZone(drive, targetPos, 0.508, 3.302);
        boolean inOuter = isShootingZone(drive, targetPos, 3.302, 3.5);
        
        String currentState;
        if (inInner) {
    currentState = "INNER";
    } else if (inOuter) {
    currentState = "OUTER";
    } else {
    currentState = "NONE";
}

        // 3. The Change Detection check
        if (!currentState.equals(previousState)) {
            switch (currentState) {
                case "INNER" -> ledSystem.setAnimation(AnimationType.Rainbow, 1);
                case "OUTER" -> ledSystem.setAnimation(AnimationType.
                Larson, 1);
                case "NONE"  -> ledSystem.setAnimation(AnimationType.ColorFlow, 1);
            }
            previousState = currentState;
        }
    }, ledSystem)
    .beforeStarting(() -> {
        // initalize
        // Determine where we are the momentthe button is pressed
        boolean inInner = isShootingZone(drive, targetPos, 0.508, 3.302);
        boolean inOuter = isShootingZone(drive, targetPos, 3.302, 3.5);
        
       
if (inInner) {
    previousState = "INNER";
} else if (inOuter) {
    previousState = "OUTER";
} else {
    previousState = "NONE";
}

        // Immediately set the LED so there is zero delay
        switch (previousState) {
            case "INNER" -> ledSystem.setAnimation(AnimationType.Rainbow, 1);
            case "OUTER" -> ledSystem.setAnimation(AnimationType.Larson, 1);
            case "NONE"  -> ledSystem.setAnimation(AnimationType.ColorFlow, 1);
        }
    });
}
  public static Command getRunMyPathCommand(String string) {
    try {
        return AutoBuilder.followPath(
            PathPlannerPath.fromPathFile(string)
        );
    } catch (Exception e) {
        DriverStation.reportError("Failed to load path: " + string, false);
        return Commands.none();
    }
}

  public static PathPlannerPath loadPath(String pathName) {
    try {
        return PathPlannerPath.fromPathFile(pathName);
    } catch (Exception e) {
        DriverStation.reportError("Failed to load path: " + pathName, false);
        return null;
    }
  }

}




