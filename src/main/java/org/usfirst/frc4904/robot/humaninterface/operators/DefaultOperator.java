package org.usfirst.frc4904.robot.humaninterface.operators;

import org.usfirst.frc4904.robot.RobotMap;
import org.usfirst.frc4904.standard.commands.CreateAndDisown;
import org.usfirst.frc4904.standard.humaninput.Operator;

// import edu.wpi.first.wpilibj2.command.InstantCommand;

public class DefaultOperator extends Operator {

    // private boolean justHeldHighCone = false;

    public DefaultOperator() {
        super("DefaultOperator");
    }

    public DefaultOperator(String name) {
        super(name);
    }

    @Override
    public void bindCommands() {}
}
