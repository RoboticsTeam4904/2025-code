package org.usfirst.frc4904.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import org.usfirst.frc4904.standard.commands.CreateOnInitialize;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezPID;
import org.usfirst.frc4904.standard.custom.motorcontrollers.SmartMotorController;

public class SwerveModule {
    private final DriveController drive;
    private final RotationController rotation;

    public SwerveModule(
        SmartMotorController driveMotor,
        SmartMotorController rotMotor,
        DutyCycleEncoder rotEncoder,
        Translation2d direction
    ) {
        drive = new DriveController(driveMotor);
        rotation = new RotationController(rotMotor, rotEncoder, direction);
    }

    public Translation2d rotToTranslation(double theta) {
        return rotation.toTranslation(theta);
    }

    public Command c_moveTo(double magnitude, double theta) {
        return new ParallelCommandGroup(
            drive.c_setMagnitude(magnitude),
            rotation.c_gotoRotation(theta)
        );
    }
}

record DriveController(SmartMotorController motor) {
    public Command c_setMagnitude(double magnitude) {
        return new RunCommand(() ->
            motor.set(magnitude / SwerveConstants.LIN_SPEED)
        );
    }
}

class RotationController {
    private static final double kP = 1; // TODO: tune
    private static final double kI = 0;
    private static final double kD = 0;

    private final SmartMotorController motor;
    private final DutyCycleEncoder encoder;

    private final Translation2d direction;

    /**
     * @param direction Direction should have a magnitude of √2, as in {@code new Translation2d(±1, ±1)}
     */
    public RotationController(
        SmartMotorController motor,
        DutyCycleEncoder encoder,
        Translation2d direction
    ) {
        this.motor = motor;
        this.encoder = encoder;

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

        // encoder readings are from 0-1 but opposite angles are equivalent
        // since we can just run the wheels backwards (see below)
        pid.enableContinuousInput(0, 0.5);

        return new ezPID(
            pid,
            this::getRotation,
            (voltage) -> {
                // run this wheel backwards if we are more than 90deg off from the target angle
                boolean flip = Math.abs(this.getRotation() - theta) > 0.25;
                setVoltage(flip ? -voltage : voltage);
            },
            theta
        );
    }

    private double getRotation() {
        return encoder.get();
    }

    private void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }
}
