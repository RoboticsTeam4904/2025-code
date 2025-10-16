package org.usfirst.frc4904.robot.humaninterface.operators;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.robot.subsystems.ElevatorSubsystem;
import org.usfirst.frc4904.robot.subsystems.VisionSubsystem.TagGroup;
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
        joystick.button11.onTrue(Component.elevator.c_intakeRaw());
        /// RAMP OUTTAKE
        joystick.button12.onTrue(Component.elevator.c_rampOuttakeRaw());

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

        /// VISION
        turnJoystick.button1.whileTrue(Component.vision.c_align(TagGroup.ANY, -1));
        turnJoystick.button2.whileTrue(Component.vision.c_align(TagGroup.ANY, 1));

        /// ODOMETRY RESETTING
        xyJoystick.button1.onTrue(new InstantCommand(() -> Component.chassis.resetOdometry()));

        /// ELEVATOR ENCODER RESETTING
        var elevatorEncoderCommand = new InstantCommand(() -> Component.elevator.setVoltage(-3, true));
        elevatorEncoderCommand.addRequirements(Component.elevator);
        joystick.button10.onTrue(elevatorEncoderCommand);
        joystick.button10.onFalse(new InstantCommand(() -> {
            Component.elevator.setVoltage(0);
            Component.elevatorEncoder.reset();
        }));
    }
}
