package org.usfirst.frc4904.robot;

import org.usfirst.frc4904.robot.sim.mechsim;
import org.usfirst.frc4904.robot.subsystems.armsubsystem;
import org.usfirst.frc4904.robot.subsystems.arm.io.armioreal;
import org.usfirst.frc4904.robot.subsystems.arm.io.armsimio;

public class robotcontainer {
    public final mechsim sim;
    private final armsubsystem arm;

    public robotcontainer(boolean isSimulation){
        if (Robot.isSimulation()){
            arm = new armsubsystem(new armsimio());
        } else{arm = new armsubsystem(new armsimio());}
            sim = new mechsim(arm);
    }
}
