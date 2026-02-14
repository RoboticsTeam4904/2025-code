package org.usfirst.frc4904.robot.sim;

import java.util.logging.Logger;

import org.usfirst.frc4904.robot.subsystems.armsubsystem;
import org.usfirst.frc4904.robot.subsystems.arm.armstate;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class mechsim {

private final Mechanism2d panel;
private final MechanismRoot2d root;
private final MechanismLigament2d  arm;

private final armsubsystem armsubsystem;

public mechsim(armsubsystem arms){
    this.armsubsystem = arms;

    this.panel = new Mechanism2d(100, 100);
    this.root = panel.getRoot("arm", 20, 10);
    this.arm = root.append(new MechanismLigament2d("arm", 95, 0, 6, new Color8Bit(Color.kAqua)));
}

public void periodic(){
    armstate.InputState currentState = armsubsystem.getState();
    if (currentState != null){
        this.arm.setAngle(currentState.currentAngleDegrees());
    //undo
    
       // System.out.println(currentState.currentAngleDegrees());
       
    }
SmartDashboard.putData("sim", panel);

}
//note to self it moves 1 degree and back to 0
}
