package org.usfirst.frc4904.robot.subsystems;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import org.usfirst.frc4904.standard.commands.CreateOnInitialize;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezControl;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezMotion;
import org.usfirst.frc4904.standard.custom.motorcontrollers.SmartMotorController;

public class CustomModule {
    private final MoveController move;
    private final RotationController rotation;

    public CustomModule(
        SmartMotorController moveMotor,
        SmartMotorController rotMotor,
        DutyCycleEncoder rotEncoder,
        Translation2d direction
    ) {
        move = new MoveController(moveMotor);
        rotation = new RotationController(rotMotor, rotEncoder, direction);
    }

    public Translation2d rotToTranslation(double theta) {
        return rotation.toTranslation(theta);
    }


    public Command c_moveTo(double magnitude, double theta) {
        return new ParallelCommandGroup(
            move.c_setMagnitude(magnitude),
            rotation.c_gotoRotation(theta)
        );
    }
}

record MoveController(SmartMotorController motor) {

    public Command c_setMagnitude(double magnitude) {
        return new RunCommand(() ->
            motor.setVoltage(
                magnitude / SwerveConstants.LIN_SPEED * SwerveConstants.MOTOR_VOLTS // TODO dubious
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

    /**
     * @param direction Direction should have a magnitude of √2, as in {@code new Translation2d(±1, ±1)}
     */
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
        PIDController pid = new PIDController(kP, kI, kD);
        pid.enableContinuousInput(0, 1);

        ezControl controller = new ezControl(
            pid,
            (pos, mPerSec) -> this.ff.calculate(mPerSec),
            0.02
        );

        TrapezoidProfile profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(SwerveConstants.ROT_SPEED, SwerveConstants.ROT_ACCEL)
        );

        return getEzMotion(
            controller,
            profile,
            new TrapezoidProfile.State(getRotation(), 0), // TODO use encoder to keep track of current m/s
            new TrapezoidProfile.State(theta, 0)
        );
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

