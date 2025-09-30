package org.usfirst.frc4904.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.usfirst.frc4904.robot.RobotMap.Component;

import java.util.function.DoubleSupplier;

class SwerveConstants {
    // TODO: get real measurements
    public static final double RPM = 1;
    public static final double GEAR_RATIO = 1;

    // meters - TODO: get real measurements
    public static final double WHEEL_RADIUS = 0.01;
    public static final double ROBOT_DIAGONAL = 1;

    // m/s
    public static final double LIN_SPEED = RPM / 60.0 / GEAR_RATIO * (2 * Math.PI * WHEEL_RADIUS);
    // turns/s
    public static final double ROT_SPEED = LIN_SPEED / (Math.PI * ROBOT_DIAGONAL);

    // TODO: get real measurements
    public static final double ROT_MOTOR_SPEED = 5;
    public static final double ROT_MOTOR_ACCEL = ROT_MOTOR_SPEED * 10;

    public static final double MOTOR_VOLTS = 12;
}

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule[] modules;

    public SwerveSubsystem(SwerveModule... modules) {
        this.modules = modules;
    }

    /**
     * Drive according to joystick inputs. {@code hypot(x, y)} should be <= 1.
     * @param x X movement from [-1, 1]
     * @param y Y movement from [-1, 1]
     * @param theta Turn speed from [-1, 1]
     */
    public void input(double x, double y, double theta) {
        Translation2d scaled = new Translation2d(x, y).times(SwerveConstants.LIN_SPEED);
        driveFieldRelative(scaled, theta * SwerveConstants.ROT_SPEED);
    }

    public Translation2d toRobotRelative(Translation2d translation) {
        // TODO account for alliance direction
        float rotation = Component.navx.getYaw();
        return translation.rotateBy(Rotation2d.fromDegrees(-rotation));
    }

    /**
     * Drive relative to the field.
     * @param translation Movement speed in meters per second
     * @param theta Rotation speed in rotations per second - not field-relative,
     *              as it represents the turning speed, not an absolute angle.
     */
    public void driveFieldRelative(Translation2d translation, double theta) {
        driveRobotRelative(toRobotRelative(translation), theta);
    }

    /**
     * Drive relative to the current angle of the robot.
     * @param translation Movement speed in meters per second
     * @param theta Rotation speed in rotations per second
     */
    public void driveRobotRelative(Translation2d translation, double theta) {
        Translation2d[] translations = new Translation2d[modules.length];
        double maxMag = SwerveConstants.LIN_SPEED;

        for (int i = 0; i < modules.length; i++) {
            Translation2d rotation = modules[i].rotToTranslation(theta);
            Translation2d sum = translation.plus(rotation);

            translations[i] = sum;
            maxMag = Math.max(sum.getNorm(), maxMag);
        }

        double norm = maxMag / SwerveConstants.LIN_SPEED;

        for (int i = 0; i < modules.length; i++) {
            Translation2d normalized = translations[i].div(norm);

            modules[i].moveTo(
                normalized.getNorm(),
                normalized.getAngle().getRotations()
            );
        }
    }

    /// COMMANDS

    /**
     * Hold a field-relative movement speed and rotation speed.
     * <p>
     * See {@link #driveFieldRelative(Translation2d, double)}
     */
    public Command c_driveFieldRelative(Translation2d translation, double theta) {
        return run(() -> driveFieldRelative(translation, theta));
    }

    /**
     * Hold a robot-relative movement speed and rotation speed.
     * <p>
     * See {@link #driveRobotRelative(Translation2d, double)}
     */
    public Command c_driveRobotRelative(Translation2d translation, double theta) {
        return run(() -> driveRobotRelative(translation, theta));
    }

    /**
     * Drive according to inputs provided by the suppliers.
     * <p>
     * See {@link #input(double, double, double)}
     */
    public Command c_input(DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta) {
        return run(() -> input(x.getAsDouble(), y.getAsDouble(), theta.getAsDouble()));
    }
}
