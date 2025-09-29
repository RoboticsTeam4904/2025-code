package org.usfirst.frc4904.standard.custom.motioncontrollers;

import edu.wpi.first.math.controller.PIDController;

public class ezControl {
    public final PIDController pid;
    public final ezFeedForward ff;

    private double setpoint;
    private double setpoint_dt;

    public ezControl(double kP, double kI, double kD, ezFeedForward ff, double errorTolerance) {
        this(kP, kI, kD, ff);
        pid.setTolerance(errorTolerance, 1);
    }

    public ezControl(double kP, double kI, double kD, ezFeedForward ff) {
        this(new PIDController(kP, kI, kD), ff);
    }

    public ezControl(PIDController pid) {
        this(pid, (pos, mPerSec) -> 0.0);
    }

    public ezControl(PIDController pid, ezFeedForward ff) {
        this.pid = pid;
        this.ff = ff;
    }

    public boolean atSetpoint() {
        return pid.atSetpoint();
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
        this.setpoint_dt = 0;
        pid.setSetpoint(setpoint);
    }

    public void updateSetpoint(double setpoint, double setpoint_dt) {
        if (setpoint != this.setpoint) {
            pid.setSetpoint(this.setpoint);
        }
        this.setpoint = setpoint;
        this.setpoint_dt = setpoint_dt;
    }

    public double calculate(double measurement, double elapsed) {
        double pidOut = pid.calculate(measurement);
        double ffOut = ff.calculate(setpoint, setpoint_dt);
        return pidOut + ffOut;
    }
}
