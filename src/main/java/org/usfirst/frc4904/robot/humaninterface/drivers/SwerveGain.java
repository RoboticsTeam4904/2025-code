package org.usfirst.frc4904.robot.humaninterface.drivers;

import edu.wpi.first.math.geometry.Translation2d;
import org.usfirst.frc4904.robot.RobotMap.HumanInput;
import org.usfirst.frc4904.robot.humaninterface.HumanInterfaceConfig;
import org.usfirst.frc4904.standard.humaninput.Driver;

public class SwerveGain extends Driver { //ALL SWERVEGAIN JOYSTICKS OUTPUT FROM -1 TO 1

    double SPEED_EXP = 2; //TODO TUNE
    double TURN_EXP = 2; //TODO TUNE

    public SwerveGain() {
        super("SwerveGain");
    }

    protected double scaleGain(double input, double exp) {
        return Math.pow(Math.abs(input), exp) * Math.signum(input);
    }

    public void bindCommands() {
        // RobotMap.HumanInput.Driver.turnJoystick.button1.onTrue(
        //     new InstantCommand(() -> RobotMap.Component.chassis.brickMode())
        // );
        // RobotMap.HumanInput.Driver.turnJoystick.button2.onTrue(
        //     new InstantCommand(() -> RobotMap.Component.chassis.zeroGyro())
        // );
    }

    public double getX() {
        double raw = HumanInput.Driver.xyJoystick.getX();
        return scaleGain(raw, SPEED_EXP);
    }

    public double getY() {
        double raw = HumanInput.Driver.xyJoystick.getY();
        return scaleGain(raw, SPEED_EXP);
    }

    public Translation2d getTrans() {
        double dead = HumanInterfaceConfig.JOYSTICK_DEADZONE;

        double rawX = HumanInput.Driver.xyJoystick.getX();
        double rawY = HumanInput.Driver.xyJoystick.getY();

        double mag = Math.hypot(rawX, rawY);

        if (mag < dead) {
            return Translation2d.kZero;
        }

        double scaled = scaleGain((mag - dead) / (1 - dead), SPEED_EXP);
        double scale = scaled / mag;

        return new Translation2d(rawX * scale, rawY * scale);
    }

    public double getTurnSpeed() {
        double raw = HumanInput.Driver.turnJoystick.getX();
        return scaleGain(raw, TURN_EXP);
    }
}
