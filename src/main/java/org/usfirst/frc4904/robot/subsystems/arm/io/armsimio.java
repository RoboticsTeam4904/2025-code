package org.usfirst.frc4904.robot.subsystems.arm.io;
import static edu.wpi.first.units.Units.Volts;

import org.usfirst.frc4904.robot.subsystems.arm.armstate;
import org.usfirst.frc4904.robot.subsystems.arm.armstate.InputState;
import org.usfirst.frc4904.robot.subsystems.arm.armstate.OutputState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;


public class armsimio implements armioreal {

    private final SingleJointedArmSim sim = new SingleJointedArmSim(DCMotor.getNeo550(1), 50, SingleJointedArmSim.estimateMOI(95, 0.05), 95, 0, 0.5*Math.PI, true, 0.25*Math.PI, 0.005, 0.005);
    public InputState getState() {
        sim.update(0.02);
        //whatup with t
        return new armstate.InputState(Units.radiansToDegrees(sim.getAngleRads()), Units.radiansToDegrees(sim.getVelocityRadPerSec()));
    
    }

    @Override
    //UNDO FOR THE LOVE OF THE MATERIAL PLANE PLEASE DONT FORGET TO UNDO
   public void setstate(OutputState output) {
       // System.out.println("voltage ="+output.voltage());
       //this below is broken somehow and the math is prob too b/c thing is going in the. wrong direction
       //output.voltage().ifPresent((volts) -> {sim.setInputVoltage(volts);});

        
//undo?????
//i think something is f***ed up with set input voltag

}
   
}