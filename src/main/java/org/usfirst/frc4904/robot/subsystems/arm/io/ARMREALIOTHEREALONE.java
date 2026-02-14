package org.usfirst.frc4904.robot.subsystems.arm.io;

import org.usfirst.frc4904.robot.subsystems.arm.armstate.InputState;
import org.usfirst.frc4904.robot.subsystems.arm.armstate.OutputState;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class ARMREALIOTHEREALONE implements armioreal {

    Spark spark = new Spark(1);
    @Override
    public InputState getState() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public void setstate(OutputState output) {
        // TODO Auto-generated method stub
        //spark.setVoltage(output.voltage());
        //return null;
    }

}
