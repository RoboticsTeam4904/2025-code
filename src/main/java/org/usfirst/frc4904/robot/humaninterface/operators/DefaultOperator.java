package org.usfirst.frc4904.robot.humaninterface.operators;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.robot.subsystems.ElevatorSubsystem;

import org.usfirst.frc4904.standard.humaninput.Operator;

public class DefaultOperator extends Operator {

    public DefaultOperator() {
        super("DefaultOperator");
    }

    public DefaultOperator(String name) {
        super(name);
    }

    @Override
    public void bindCommands() {
        var joystick = RobotMap.HumanInput.Operator.joystick;
        var xyJoystick = RobotMap.HumanInput.Driver.xyJoystick;
        var turnJoystick = RobotMap.HumanInput.Driver.turnJoystick;

        /// ELEVATOR SETPOINTS
        joystick.button7.onTrue(Component.elevator.c_gotoPosition(ElevatorSubsystem.Position.INTAKE));
        joystick.button8.onTrue(Component.elevator.c_gotoPosition(ElevatorSubsystem.Position.L2));
        joystick.button9.onTrue(Component.elevator.c_gotoPosition(ElevatorSubsystem.Position.L3));

        /// INTAKE
        joystick.button11.onTrue(Component.elevator.c_intake());
        /// RAMP OUTTAKE
        joystick.button12.onTrue(Component.elevator.c_rampOuttake());

        /// MANUAL RAMP CONTROL
        joystick.button3.onTrue(Component.ramp.c_forward());
        joystick.button5.onTrue(Component.ramp.c_backward());
        joystick.button3.onFalse(Component.ramp.c_stop());
        joystick.button5.onFalse(Component.ramp.c_stop());

        /// MANUAL OUTTAKE CONTROL
        joystick.button4.onTrue(Component.outtake.c_forward());
        joystick.button6.onTrue(Component.outtake.c_backward());
        joystick.button4.onFalse(Component.outtake.c_stop());
        joystick.button6.onFalse(Component.outtake.c_stop());

        /// ODOMETRY RESETTING
        xyJoystick.button1.onTrue(c_resetOdometry());

        /// ELEVATOR ENCODER RESETTING
        joystick.button10.whileTrue(c_manualElevatorZero());
    }
}
