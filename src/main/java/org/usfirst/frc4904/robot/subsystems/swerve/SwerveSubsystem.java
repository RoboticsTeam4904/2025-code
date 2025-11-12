package org.usfirst.frc4904.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.standard.commands.NoOp;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

class SwerveConstants {
    // TODO: get real measurements
    public static final double LIN_RPM = 6380;
    public static final double LIN_GEAR_RATIO = 5.1;

    // meters - TODO: get real measurements
    public static final double WHEEL_RADIUS = 0.07;
    public static final double ROBOT_DIAGONAL = 1.15;

    // m/s
    public static final double LIN_SPEED = LIN_RPM / 60.0 / LIN_GEAR_RATIO * (2 * Math.PI * WHEEL_RADIUS);
    // turns/s
    public static final double ROT_SPEED = LIN_SPEED / (Math.PI * ROBOT_DIAGONAL);

    // TODO: get real measurements
    public static final double ROT_RPM = 11000;
    public static final double ROT_GEAR_RATIO = 46.42;

    public static final double ROT_MOTOR_SPEED = ROT_RPM / 60.0 / ROT_GEAR_RATIO;
    public static final double ROT_MOTOR_ACCEL = ROT_MOTOR_SPEED * 12;
}

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule[] modules;

    public SwerveSubsystem(SwerveModule... modules) {
        this.modules = modules;
    }

    /**
     * Drive according to joystick inputs. {@code hypot(x, y)} should be <= 1.
     * @param translation X/Y movement from [-1, 1]
     * @param theta Turn speed from [-1, 1]
     */
    public void input(Translation2d translation, double theta) {
        // System.out.println("x: " + x + " y: " + y + " theta: " + theta);

        Translation2d scaled = translation.times(SwerveConstants.LIN_SPEED);
        driveFieldRelative(scaled, theta * SwerveConstants.ROT_SPEED);
    }

    public Translation2d toRobotRelative(Translation2d translation) {
        // TODO account for alliance direction
        double rotation = Component.navx.getYaw() + 90;
        return translation.rotateBy(Rotation2d.fromDegrees(rotation));
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

            double magnitude = normalized.getNorm();
            modules[i].moveTo(
                magnitude,
                magnitude > 0 ? normalized.getAngle().getRotations() : 0
            );
        }
    }

    public void driveRobotRelative(double x, double y, double theta) {
        driveRobotRelative(new Translation2d(x, y), theta);
    }

    public void stop() {
        driveRobotRelative(0, 0, 0);
    }

    @Override
    public void periodic() {
        for (var module : modules) module.periodic();
    }

    /// COMMANDS

    /**
     * Stop the wheels.
     */
    public Command c_stop() {
        return runOnce(this::stop);
    }

    /**
     * Hold a field-relative movement speed and rotation speed.
     * <p>
     * See {@link #driveFieldRelative(Translation2d, double)}
     */
    public Command c_driveFieldRelative(Translation2d translation, double theta) {
        return run(() -> driveFieldRelative(translation, theta));
    }

    /**
     * Hold a field-relative movement speed and rotation speed.
     * <p>
     * See {@link #driveFieldRelative(Translation2d, double)}
     */
    public Command c_driveFieldRelative(double x, double y, double theta) {
        Translation2d translation = new Translation2d(x, y);
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
     * Hold a robot-relative movement speed and rotation speed.
     * <p>
     * See {@link #driveRobotRelative(Translation2d, double)}
     */
    public Command c_driveRobotRelative(double x, double y, double theta) {
        Translation2d translation = new Translation2d(x, y);
        return run(() -> driveRobotRelative(translation, theta));
    }

    /**
     * Drive according to inputs provided by the suppliers.
     * <p>
     * See {@link #input(Translation2d, double)}
     */
    public Command c_input(Supplier<Translation2d> trans, DoubleSupplier theta) {
        return run(() -> input(trans.get(), theta.getAsDouble()));
    }

    public void resetOdometry() {
        Component.navx.zeroYaw();
    }

    // TODO remove

    public Command setMotorBrake(Boolean brake) {
        return new NoOp();
    }

    public Command getAutonomousCommand(String path, Boolean setOdom, Boolean flipSide) {
        return new NoOp();
    }
}
