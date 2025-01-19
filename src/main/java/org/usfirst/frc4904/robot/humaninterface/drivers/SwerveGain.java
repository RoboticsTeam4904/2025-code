package org.usfirst.frc4904.robot.humaninterface.drivers;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.humaninput.Driver;

public class SwerveGain extends Driver { //ALL SWERVEGAIN JOYSTICKS OUTPUT FROM -1 TO 1

    double SPEED_EXP = 2; //TODO TUNE
    double TURN_EXP = 2; //TODO TUNE
    static boolean button1 = false;

    public SwerveGain() {
        super("SwerveGain");
    }

    protected double scaleGain(double input, double exp) {
        return Math.pow(Math.abs(input), exp) * Math.signum(input);
    }

    public void bindCommands() {
        var xbox = RobotMap.HumanInput.Driver.xbox;

        xbox.x().onTrue(new InstantCommand(() -> RobotMap.Component.chassis.brickMode()));
        xbox.y().onTrue(new InstantCommand(() -> RobotMap.Component.chassis.zeroGyro()));
    }

    public double getX() {
        double raw = RobotMap.HumanInput.Driver.xbox.getLeftX();
        return scaleGain(raw, SPEED_EXP);
    }

    public double getY() {
        double raw = RobotMap.HumanInput.Driver.xbox.getLeftY();
        return scaleGain(raw, SPEED_EXP);
    }

    public double getTurnSpeed() {
        double raw = RobotMap.HumanInput.Driver.xbox.getRightX();
        return scaleGain(raw, TURN_EXP);
    }
    // public boolean getButton1Pressed() {
    //     RobotMap.HumanInput.Driver.turnJoystick.button1.onTrue(
    //         new InstantCommand(() -> SwerveGain.button1 = true)
    //     );
    //     RobotMap.HumanInput.Driver.turnJoystick.button1.onFalse(
    //         new InstantCommand(() -> SwerveGain.button1 = false)
    //     );
    //     return SwerveGain.button1;
    // }
}
