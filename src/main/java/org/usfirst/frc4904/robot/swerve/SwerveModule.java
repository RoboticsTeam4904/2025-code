package org.usfirst.frc4904.robot.swerve;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;

import org.usfirst.frc4904.standard.Util;
import org.usfirst.frc4904.standard.custom.motorcontrollers.SmartMotorController;

public class SwerveModule {
    private final DriveController drive;
    private final RotationController rotation;

    private double magnitude = 0;
    private double theta = 0;

    public SwerveModule(
        SmartMotorController driveMotor,
        SmartMotorController rotMotor,
        SparkAbsoluteEncoder rotEncoder,
        Translation2d direction
    ) {
        drive = new DriveController(driveMotor);
        rotation = new RotationController(rotMotor, rotEncoder, direction);
    }

    public Translation2d rotToTranslation(double theta) {
        return rotation.toTranslation(theta);
    }

    public void moveTo(double magnitude, double theta) {
        this.magnitude = magnitude;
        if (magnitude > 0) this.theta = theta;
    }

    public void periodic() {
        // TODO run this faster than 50hz - run pid on motor
        boolean flip = rotation.rotateToward(theta);
        drive.setMagnitude(flip ? -magnitude : magnitude);
    }
}

record DriveController(SmartMotorController motor) {
    public void setMagnitude(double magnitude) {
        motor.set(magnitude / SwerveConstants.LIN_SPEED);
    }
}

class RotationController {
    private static final double kP = 40;
    private static final double kI = 0;
    private static final double kD = 0;

    private static final double MAX_VOLTAGE = 8;

    public final SmartMotorController motor;
    private final SparkAbsoluteEncoder encoder;

    private final Translation2d direction;

    private final PIDController pid;

    public RotationController(
        SmartMotorController motor,
        SparkAbsoluteEncoder encoder,
        Translation2d direction
    ) {
        this.motor = motor;
        this.encoder = encoder;

        this.direction = direction.div(direction.getNorm());

        this.pid = new PIDController(kP, kI, kD);
        // encoder readings are from 0-1 but opposite angles are equivalent
        // since we can just run the wheels backwards
        this.pid.enableContinuousInput(0, 0.5);
    }

    public Translation2d toTranslation(double theta) {
        return direction.times(theta);
    }

    private double getRotation() {
        // flip so that positive is counterclockwise
        return 1 - encoder.getPosition();
    }

    private void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    /**
     * @return True if the wheel is currently more than halfway off the target
     *         and therefore should drive in the opposite direction.
     */
    public boolean rotateToward(double theta) {
        double current = getRotation();
        double voltage = pid.calculate(current, theta);
        setVoltage(Util.clamp(voltage, -MAX_VOLTAGE, MAX_VOLTAGE));

        double dist = Math.abs(theta - current);
        return dist > 0.25 && dist < 0.75;
    }
}
