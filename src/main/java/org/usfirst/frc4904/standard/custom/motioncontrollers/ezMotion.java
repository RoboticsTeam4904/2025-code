package org.usfirst.frc4904.standard.custom.motioncontrollers;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.usfirst.frc4904.standard.custom.motioncontrollers.ezMotion.SetpointSupplier.EndSignal;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ezMotion extends Command {
    public ezControl control;
    public DoubleConsumer processVariable;
    public Double initialTimestamp;
    public DoubleSupplier feedback;

    private double setpoint;
    private double setpoint_dt;

    public Supplier<SetpointSupplier<Pair<Double, Double>>> setpointDealerDealer;
    public SetpointSupplier<Pair<Double, Double>> setpointDealer = null;

    public boolean finishOnArrival;

    public ezMotion(ezControl control,
                    DoubleSupplier feedback,
                    DoubleConsumer processVariable,
                    Supplier<SetpointSupplier<Pair<Double, Double>>> setpointDealerDealer,
                    boolean finishOnArrival,
                    Subsystem... requirements) {

        addRequirements(requirements);
        this.control = control;
        this.processVariable = processVariable;
        this.feedback = feedback;
        this.setpointDealerDealer = setpointDealerDealer;
        this.finishOnArrival = finishOnArrival;
    }

    public ezMotion(ezControl control,
                    DoubleSupplier feedback, DoubleConsumer processVariable,
                    Supplier<SetpointSupplier<Pair<Double, Double>>> setpointDealerDealer,
                    Subsystem... requirements)
    { this(control, feedback, processVariable, setpointDealerDealer, true, requirements); }

    public ezMotion(ezControl control,
                    DoubleSupplier feedback,
                    DoubleConsumer processVariable,
                    SetpointSupplier<Pair<Double, Double>> setpointDealer,
                    boolean finishOnArrival,
                    Subsystem... requirements)
    { this(control, feedback, processVariable, () -> setpointDealer, finishOnArrival, requirements); }

    public ezMotion(ezControl control,
                    DoubleSupplier feedback,
                    DoubleConsumer processVariable,
                    SetpointSupplier<Pair<Double, Double>> setpointDealer,
                    Subsystem... requirements)
    { this(control, feedback, processVariable, () -> setpointDealer, requirements); }

    public double getElapsedTime() {
        return Timer.getFPGATimestamp() - initialTimestamp;
    }

    public boolean atLatestSetpoint() {
        return this.control.atSetpoint();
    }

    @Override
    public void initialize() {
        setpointDealer = setpointDealerDealer.get();
        initialTimestamp = Timer.getFPGATimestamp();
    }

    private boolean finished = false;

    @Override
    public void execute() {
        finished = false;

        try {
            Pair<Double, Double> setpoints = setpointDealer.apply(getElapsedTime());
            setpoint = setpoints.getFirst();
            setpoint_dt = setpoints.getSecond();
        } catch (EndSignal e) {
            finished = true;
        }

        // TODO set `finished = true` when we arrive at the setpoint
        //      (setpoint as in the FINAL destination, which is different than the 'setpoint' class field)

        control.updateSetpoint(setpoint, setpoint_dt);
        double controlEffort = control.calculate(feedback.getAsDouble(), getElapsedTime());
        processVariable.accept(controlEffort);
    }

    @Override
    public boolean isFinished() { return finishOnArrival && finished; }

    @FunctionalInterface
    public interface SetpointSupplier<R> {
        class EndSignal extends Throwable {}

        R apply(double num) throws EndSignal;
    }
}
