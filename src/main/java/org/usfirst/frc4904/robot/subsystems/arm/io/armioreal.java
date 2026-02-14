package org.usfirst.frc4904.robot.subsystems.arm.io;

import org.usfirst.frc4904.robot.subsystems.arm.armstate;

public interface armioreal {
    public armstate.InputState getState();
    public void setstate(armstate.OutputState output);
}
