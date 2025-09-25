package org.usfirst.frc4904.standard.custom.motioncontrollers;

import edu.wpi.first.math.controller.PIDController;

import java.util.function.BiFunction;

public class ezControl implements BiFunction<Double, Double, Double> {
    private final ezControlMethod controller;
    private double setpoint;
    private double setpoint_dt;

    public ezControl(double kP, double kI, double kD, ezFeedForward F) {
        this(new PIDController(kP, kI, kD), F);
    }

    public ezControl(PIDController pid, ezFeedForward F) {
        // 0.05 = default setpoint error tolerance
        this(pid, F, 0.05);
    }

    public ezControl(double kP, double kI, double kD, ezFeedForward F, double errorTolerance) {
        this(new PIDController(kP, kI, kD), F, errorTolerance);
    }

    public ezControl(PIDController pid, ezFeedForward F, double errorTolerance) {
        pid.setTolerance(errorTolerance, 1);
        this.controller = new ezControlMethod(pid, F);
    }

    public boolean atSetpoint() {
        return this.controller.pid.atSetpoint();
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
        this.setpoint_dt = 0;
        this.controller.pid.setSetpoint(setpoint);
    }

    public void updateSetpoint(double setpoint, double setpoint_dt) {
        if (setpoint != this.setpoint) {
            // slightly expensive call that clears integral
            this.controller.pid.setSetpoint(this.setpoint);
        }
        this.setpoint = setpoint;
        this.setpoint_dt = setpoint_dt;
    }

    public void setIntegratorRange(double kIMin, double kIMax) {
        this.controller.pid.setIntegratorRange(kIMin, kIMin);
    }

    public double calculate(double measurement, double elapsed) {
        // FIXME, revert logging
        // SmartDashboard.putNumber("setpoint", this.setpoint);
        // SmartDashboard.putNumber("setpoint_dt", this.setpoint_dt);

        double pidOut = this.controller.pid.calculate(measurement);
        // System.out.println(pidOut);
        // SmartDashboard.putNumber("Feedback", measurement);
        // SmartDashboard.putNumber("PID out", pidOut);
        return pidOut + this.controller.F.calculate(this.setpoint, this.setpoint_dt);
    }

    // very similar to TrapezoidProfile.State
    @Override
    public Double apply(Double measurement, Double elapsedPeriod) {
        return calculate(measurement, elapsedPeriod);
    }

    public record ezControlMethod(PIDController pid, ezFeedForward F) {}

    public ezControlMethod getControl() {
        return this.controller;
    }

}
