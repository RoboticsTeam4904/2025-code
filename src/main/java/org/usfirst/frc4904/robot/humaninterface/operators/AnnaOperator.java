package org.usfirst.frc4904.robot.humaninterface.operators;

import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.robot.RobotMap.HumanInput;
import org.usfirst.frc4904.robot.subsystems.ElevatorSubsystem.Position;
import org.usfirst.frc4904.standard.humaninput.Operator;

public class AnnaOperator extends Operator {

    public AnnaOperator() {
        super("AnnaOperator");
    }

    public AnnaOperator(String name) {
        super(name);
    }

    @Override
    public void bindCommands() {
        var joystick = HumanInput.Operator.joystick;
        var xyJoystick = HumanInput.Driver.xyJoystick;
        var turnJoystick = HumanInput.Driver.turnJoystick;

        /// ELEVATOR SETPOINTS
        joystick.button7.onTrue(Component.elevator.c_gotoPosition(Position.INTAKE));
        joystick.button8.onTrue(Component.elevator.c_gotoPosition(Position.L2));
        joystick.button9.onTrue(Component.elevator.c_gotoPosition(Position.L3));

        /// INTAKE
        joystick.button11.onTrue(Component.elevator.c_intake());

        /// MANUAL RAMP CONTROL
        joystick.button6.onTrue(Component.ramp.c_forward());
        joystick.button4.onTrue(Component.ramp.c_backward());
        joystick.button6.onFalse(Component.ramp.c_stop());
        joystick.button4.onFalse(Component.ramp.c_stop());

        /// MANUAL OUTTAKE CONTROL
        joystick.button3.onTrue(Component.outtake.c_backward());
        joystick.button5.onTrue(Component.outtake.c_forward());
        joystick.button3.onFalse(Component.outtake.c_stop());
        joystick.button5.onFalse(Component.outtake.c_stop());

      
        /// ODOMETRY RESETTING
        xyJoystick.button1.onTrue(c_resetOdometry());

        /// ELEVATOR ENCODER RESETTING
        joystick.button10.whileTrue(c_manualElevatorZero());
    }
}
