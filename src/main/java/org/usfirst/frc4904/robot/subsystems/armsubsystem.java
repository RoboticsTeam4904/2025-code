package org.usfirst.frc4904.robot.subsystems;

import java.util.Optional;

import org.usfirst.frc4904.robot.subsystems.arm.armstate;
import org.usfirst.frc4904.robot.subsystems.arm.armstate.GoalState;
import org.usfirst.frc4904.robot.subsystems.arm.io.armioreal;
import org.usfirst.frc4904.robot.subsystems.arm.io.armsimio;

import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class armsubsystem extends SubsystemBase{

    public final armioreal io;

    private final PhoenixPIDController controller;
    
    private final ArmFeedforward ff;

    private armstate.InputState currentstate;

    private armstate.GoalState goal = new  armstate.GoalState(90);
   
   
   
    public armsubsystem(armioreal armIO) {
        this.io = armIO;
        this.controller = new PhoenixPIDController(0.01, 0, 0);
        this.ff= new ArmFeedforward(0, 2.657, 0);
    }

    @Override
    public void periodic(){
       
      
        if(DriverStation.isEnabled()){
            currentstate = this.io.getState();
//ITS BROKEN HERE AS BEST I CAN TELL
            double effort = this.controller.calculate(currentstate.currentAngleDegrees(), goal.position(), 0);
           double feedforward = this.ff.calculate(Units.degreesToRadians(currentstate.currentAngleDegrees()), Units.degreesToRadians(currentstate.currentVelocityDegreesPerSecond()));
            effort += feedforward;

            this.io.setstate(new armstate.OutputState(Optional.of(effort)));
           // System.out.println(effort);
            System.out.println("it knows where it is"+currentstate.currentAngleDegrees());
             System.out.println("the missile knows wher it isnt:"+goal.position());
             
             //returns as NaN
             //WHY DOESNT THE ARM SIM HAVE A VALID CURRENT ANGLE??????
    }
}
    public armstate.InputState getState(){
        return  this.currentstate;
    }
    public Command setarmpos(double degrees){


        return new InstantCommand(()->this.goal = new GoalState(degrees));
    }
    
}