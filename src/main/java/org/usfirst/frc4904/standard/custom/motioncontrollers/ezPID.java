package org.usfirst.frc4904.standard.custom.motioncontrollers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class ezPID extends Command {
    public final PIDController pid;

    private final DoubleSupplier getCurrent;
    private final DoubleConsumer processValue;

    public final double goal;

    public ezPID(
        PIDController pid,
        DoubleSupplier getCurrent,
        DoubleConsumer processValue,
        double goal,
        Subsystem... requirements
    ) {
        this.pid = pid;
        this.getCurrent = getCurrent;
        this.processValue = processValue;
        this.goal = goal;

        addRequirements(requirements);
    }

    @Override
    public void execute() {
        double current = getCurrent.getAsDouble();
        processValue.accept(pid.calculate(current, goal));
    }

    public boolean atGoal() {
        return pid.atSetpoint();
    }

    @Override
    public boolean isFinished() {
        return atGoal();
    }
}
