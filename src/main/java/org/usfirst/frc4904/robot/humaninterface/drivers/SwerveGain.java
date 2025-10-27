package org.usfirst.frc4904.robot.humaninterface.drivers;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.usfirst.frc4904.robot.RobotMap;
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
        double raw = RobotMap.HumanInput.Driver.xyJoystick.getX();
        return scaleGain(raw, SPEED_EXP);
    }

    public double getY() {
        double raw = RobotMap.HumanInput.Driver.xyJoystick.getY();
        return scaleGain(raw, SPEED_EXP);
    }

    public Translation2d getTrans() {
        double dead = HumanInterfaceConfig.JOYSTICK_DEADZONE;

        double rawX = RobotMap.HumanInput.Driver.xyJoystick.getX();
        double rawY = RobotMap.HumanInput.Driver.xyJoystick.getY();

        double mag = Math.hypot(rawX, rawY);

        if (mag < dead) {
            return new Translation2d((rawX / mag) * 0.01, (rawY / mag) * 0.01);
        }

        double ratio = 1 / (1 - dead);

        return new Translation2d(
            scaleGain((rawX - dead) * ratio, SPEED_EXP),
            scaleGain((rawY - dead) * ratio, SPEED_EXP)
        );
    }

    public double getTurnSpeed() {
        double raw = RobotMap.HumanInput.Driver.turnJoystick.getX();
        return scaleGain(raw, TURN_EXP);
    }
}
