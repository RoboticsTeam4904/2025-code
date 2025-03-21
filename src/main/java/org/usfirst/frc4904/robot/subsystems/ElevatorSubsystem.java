package org.usfirst.frc4904.robot.subsystems;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

import java.util.HashMap;
import java.util.function.DoubleSupplier;

import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.standard.commands.CreateOnInitialize;
import org.usfirst.frc4904.standard.commands.NoOp;
import org.usfirst.frc4904.standard.custom.CustomEncoder;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezControl;
import org.usfirst.frc4904.standard.custom.motioncontrollers.ezMotion;
import org.usfirst.frc4904.standard.custom.motorcontrollers.SmartMotorController;

public class ElevatorSubsystem extends MultiMotorSubsystem {

    // TODO TUNING: elevator PID
    public static final double kS = 0;
    public static final double kV = 2;
    public static final double kA = 0.1;
    public static final double kG = 0.2;

    public static final double kP = 0.05;
    public static final double kI = 0;
    public static final double kD = 0.1;

    public static final double MAX_VEL = 1;
    public static final double MAX_ACCEL = 1;

    public static final double MIN_HEIGHT = 0;
    public static final double MAX_HEIGHT = 5;

    public final ElevatorFeedforward feedforward;
    public final CustomEncoder encoder;

    // make sure that all values defined in this enum are added to the 'positions' map in the constructor
    public enum Position {
        INTAKE,
        // L1,
        L2,
        L3,
        // L4
    }

    public static HashMap<Position, Double> positions = new HashMap<>();

    // possible helpful https://www.chiefdelphi.com/t/using-encoder-to-drive-a-certain-distance/147219/2
    public ElevatorSubsystem(SmartMotorController motor1, SmartMotorController motor2, CustomEncoder encoder) {
        super(
            new SmartMotorController[] { motor1, motor2 },
            new double[] { 1, 1 },
            -5
        );
        this.feedforward = new ElevatorFeedforward(kS, kG, kV, kA);
        this.encoder = encoder;

        positions.put(Position.INTAKE, 0.0);
        // positions.put(Position.L1, 1.0);

        // TODO IMPORTANT: tune more accurately
        /* TODO */ positions.put(Position.L2, 5.27 / 1.09); // TODO worst thing since sliced bread
        /* TODO */ positions.put(Position.L3, 8.47 / 1.09); // TODO worst thing since sliced bread
        // TODO IMPORTANT: tune more accurately

        // positions.put(Position.L4, 4.0);

        for (var pos : Position.values()) {
            if (positions.get(pos) == null) {
                System.err.println(
                    "ElevatorSubsystem.Position." +
                    pos.name() +
                    " is not defined in 'positions' map"
                );
            }
        }
    }

    /**
     * @return The current height of the elevator in Magical Encoder Units™
     */
    public double getHeight() {
        return encoder.get();
    }

    /** Intake at the current elevator position */
    public Command c_intakeRaw() {
        return new SequentialCommandGroup(
            new ParallelDeadlineGroup(
                new WaitCommand(0.8),
                Component.ramp.c_forward()
            ),
            new ParallelDeadlineGroup(
                new WaitCommand(0.35),
                Component.ramp.c_forward(),
                Component.outtake.c_forward()
            ),
            new ParallelCommandGroup(
                Component.ramp.c_stop(),
                Component.outtake.c_stop()
            )
        );
    }

    /** Outtake at the current elevator position */
    public Command c_outtakeRaw() {
        return new SequentialCommandGroup(
            new ParallelDeadlineGroup(
                new WaitCommand(0.5),
                Component.outtake.c_forward()
            ),
            Component.outtake.c_stop()
        );
    }

    /** Outtake at the current elevator position */
    public Command c_rampOuttakeRaw() {
        return new SequentialCommandGroup(
            new ParallelDeadlineGroup(
                new WaitCommand(1),
                Component.outtake.c_backward(),
                Component.ramp.c_backward()
            ),
            new ParallelCommandGroup(
                Component.outtake.c_stop(),
                Component.ramp.c_stop()
            )
        );
    }

    /** Go to the intake position and then intake */
    public Command c_intake() {
        return new SequentialCommandGroup(
            c_gotoPosition(Position.INTAKE),
            new ParallelDeadlineGroup(
                c_intakeRaw(),
                c_controlVelocity(() -> 0)
            )
        );
    }

    /** Go to the intake position and then ramp outtake */
    public Command c_rampOuttake() {
        return new SequentialCommandGroup(
            c_gotoPosition(Position.INTAKE),
            new ParallelDeadlineGroup(
                c_rampOuttakeRaw(),
                c_controlVelocity(() -> 0)
            )
        );
    }

    /** Go to the specified position and then outtake */
    public Command c_outtakeAtPosition(Position pos) {
        return new SequentialCommandGroup(
            c_gotoPosition(pos),
            new ParallelDeadlineGroup(
                c_outtakeRaw(),
                c_controlVelocity(() -> 0)
            )
        );
    }

    public Command c_controlVelocity(DoubleSupplier metersPerSecDealer) {
        // if (
        //     (this.getDistance() >= MAX_HEIGHT && metersPerSecDealer.getAsDouble() < 0) ||
        //     (this.getDistance() <= MIN_HEIGHT && metersPerSecDealer.getAsDouble() > 0)
        // ) {
        //     return this.c_stop();
        // }

        var cmd = this.run(() -> {
            var ff = this.feedforward.calculate(metersPerSecDealer.getAsDouble());
            SmartDashboard.putNumber("feedforward", ff);
            this.setVoltage(ff);
        });
        cmd.setName("elevator - c_controlVelocity");

        return cmd;
    }

    public Command c_gotoPosition(Position pos) {
        Double height = positions.get(pos);

        if (height == null) {
            System.err.println("Tried to go to elevator setpoint that does not exist: " + pos.toString());
            return new NoOp();
        }

        return c_gotoHeight(height);
    }

    public Command c_gotoHeight(double height) {
        return new CreateOnInitialize(() -> this.getRawHeightCommand(height));
    }

    private Command getRawHeightCommand(double height) {
        ezControl controller = new ezControl(
            kP, kI, kD,
            (position, velocityMetersPerSec) -> this.feedforward.calculate(velocityMetersPerSec),
            0.02
        );

        TrapezoidProfile profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(MAX_VEL, MAX_ACCEL)
        );

        Command cmd = getEzMotion(
            controller,
            profile,
            new TrapezoidProfile.State(getHeight(), 0), // TODO why are we assuming the velocity is 0
            new TrapezoidProfile.State(height, 0)
        );
        cmd.setName("elevator - c_gotoHeight");
        return cmd;
    }

    private ezMotion getEzMotion(
        ezControl controller,
        TrapezoidProfile profile,
        TrapezoidProfile.State current,
        TrapezoidProfile.State goal
    ) {
        return new ezMotion(
            controller,
            this::getHeight,
            this::setVoltage,
            (double t) -> {
                TrapezoidProfile.State result = profile.calculate(t, current, goal);
                return new Pair<>(result.position, result.velocity);
            },
            this
        ) {
            @Override
            public void cancel() {
                setVoltage(0);
            }
        };
    }
}
