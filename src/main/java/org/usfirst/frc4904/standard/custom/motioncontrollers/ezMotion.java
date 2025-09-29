package org.usfirst.frc4904.standard.custom.motioncontrollers;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class ezMotion extends Command {
    public final ezControl control;

    private final DoubleSupplier getCurrent;
    private final DoubleConsumer processValue;
    private final SetpointSupplier setpointDealer;

    public double initialTimestamp;

    private double setpoint;
    private double setpoint_dt;

    public ezMotion(
        ezControl control,
        DoubleSupplier getCurrent,
        DoubleConsumer processValue,
        SetpointSupplier setpointDealer,
        Subsystem... requirements
    ) {
        this.control = control;
        this.getCurrent = getCurrent;
        this.processValue = processValue;
        this.setpointDealer = setpointDealer;

        addRequirements(requirements);
    }

    public double getElapsedTime() {
        return Timer.getFPGATimestamp() - initialTimestamp;
    }

    @Override
    public void initialize() {
        initialTimestamp = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        double elapsed = getElapsedTime();

        Pair<Double, Double> setpoints = setpointDealer.get(elapsed);
        setpoint = setpoints.getFirst();
        setpoint_dt = setpoints.getSecond();

        control.updateSetpoint(setpoint, setpoint_dt);
        double controlEffort = control.calculate(getCurrent.getAsDouble(), elapsed);
        processValue.accept(controlEffort);
    }

    @FunctionalInterface
    public interface SetpointSupplier {
        Pair<Double, Double> get(double elapsed);
    }
}
