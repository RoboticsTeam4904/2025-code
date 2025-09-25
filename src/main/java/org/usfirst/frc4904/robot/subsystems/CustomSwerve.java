package org.usfirst.frc4904.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.commands.CreateOnInitialize;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezControl;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezMotion;
import org.usfirst.frc4904.standard.custom.motorcontrollers.SmartMotorController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

class SwerveConstants {
    public static final double RPM = 1;
    public static final double GEAR_RATIO = 1;

    public static final double WHEEL_R = 1;
    public static final double ROBOT_D = 1;

    public static final double LIN_SPEED = RPM / 60.0 / GEAR_RATIO * (2 * Math.PI * WHEEL_R);
    public static final double ROT_SPEED = LIN_SPEED / (Math.PI * ROBOT_D);

    public static final double MOTOR_VOLTS = 12;
}

public class CustomSwerve extends SubsystemBase {
    private final CustomModule[] modules;

    public CustomSwerve(CustomModule[] modules) {
        this.modules = modules;
    }

    public Command c_input(double x, double y, double theta) {
        Translation2d scaled = new Translation2d(x, y).times(SwerveConstants.LIN_SPEED);
        Translation2d translation = robotRelative(scaled);

        return c_drive(translation, theta);
    }

    public Translation2d robotRelative(Translation2d translation) {
        float rotation = RobotMap.Component.navx.getYaw();

        return translation.rotateBy(Rotation2d.fromDegrees(-rotation)); // TODO PROBABLY WRONG
    }

    public Command c_drive(Translation2d translation, double theta) {
        Translation2d[] translations = new Translation2d[modules.length];
        double maxMag = 0.0;

        for (int i = 0; i < modules.length; i++) {
            Translation2d rotTrns = modules[i].rotation.toTranslation(theta);
            Translation2d modTrns = translation.plus(rotTrns);

            translations[i] = modTrns;
            maxMag = Math.max(modTrns.getNorm(), maxMag);
        }

        double norm = Math.max(maxMag, SwerveConstants.LIN_SPEED) / SwerveConstants.LIN_SPEED;
        Command[] commands = new Command[modules.length];

        for (int i = 0; i < modules.length; i++) {
            Translation2d modTrns = translations[i].div(norm);

            commands[i] = modules[i].c_moveTo(
                modTrns.getNorm(),
                modTrns.getAngle().getRotations()
            );
        }

        return new ParallelCommandGroup(commands);
    }
}

class CustomModule {
    public final MoveController move;
    public final RotationController rotation;

    public CustomModule(MoveController move, RotationController rotation) {
        this.move = move;
        this.rotation = rotation;
    }

    public Command c_moveTo(double magnitude, double theta) {
        return new ParallelCommandGroup(
            move.c_setMagnitude(magnitude),
            rotation.c_gotoRotation(theta)
        );
    }
}

class MoveController {
    SmartMotorController controller;

    public MoveController(SmartMotorController controller) {
        this.controller = controller;
    }

    public Command c_setMagnitude(double magnitude) {
        return new RunCommand(() ->
            controller.setVoltage(
                magnitude / SwerveConstants.LIN_SPEED * SwerveConstants.MOTOR_VOLTS
            )
        );
    }
}

class RotationController {
    // TODO tune
    private static final double kP = 5;
    private static final double kI = 0;
    private static final double kD = 0;
    private static final double kS = 0;
    private static final double kV = 0;

    private final SmartMotorController controller;
    private final DutyCycleEncoder encoder;

    private final SimpleMotorFeedforward ff;

    private final Translation2d direction;

    public RotationController(
        SmartMotorController controller,
        DutyCycleEncoder encoder,
        Translation2d direction
    ) {
        this.controller = controller;
        this.encoder = encoder;

        this.ff = new SimpleMotorFeedforward(kS, kV);

        this.direction = direction.div(Math.sqrt(2));
    }

    public Translation2d toTranslation(double theta) {
        return direction.times(theta);
    }

    public Command c_gotoRotation(double theta) {
        return new CreateOnInitialize(() -> this.getRotCommand(theta));
    }

    private Command getRotCommand(double theta) {
        ezControl controller = new ezControl(
            kP, kI, kD,
            (pos, mPerSec) -> this.ff.calculate(mPerSec),
            0.02
        );

        TrapezoidProfile profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(SwerveConstants.ROT_SPEED, SwerveConstants.ROT_SPEED) // TODO accel?
        );

        Command command = getEzMotion(
            controller,
            profile,
            new TrapezoidProfile.State(getRotation(), 0), // TODO use encoder to keep track of current m/s
            new TrapezoidProfile.State(theta, 0)
        );

        return command;
    }

    private double getRotation() { // TODO doesn't handle 1->0 jumps
        return encoder.get();
    }

    private void setVoltage(double voltage) {
        controller.setVoltage(voltage);
    }

    private ezMotion getEzMotion(
        ezControl controller,
        TrapezoidProfile profile,
        TrapezoidProfile.State current,
        TrapezoidProfile.State goal
    ) {
        return new ezMotion(
            controller,
            this::getRotation,
            this::setVoltage,
            (double t) -> {
                TrapezoidProfile.State result = profile.calculate(t, current, goal);
                return new Pair<>(result.position, result.velocity);
            }
        ) {
            @Override
            public void end(boolean interrupted) {
                setVoltage(0);
            }
        };
    }
}
