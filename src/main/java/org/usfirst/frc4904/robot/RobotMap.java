package org.usfirst.frc4904.robot;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLED;
import org.usfirst.frc4904.robot.subsystems.ElevatorSubsystem;
import org.usfirst.frc4904.robot.subsystems.MotorSubsystem;
import org.usfirst.frc4904.robot.swerve.SwerveModule;
import org.usfirst.frc4904.robot.swerve.SwerveSubsystem;
import org.usfirst.frc4904.standard.custom.CustomEncoder;
import org.usfirst.frc4904.standard.custom.controllers.CustomCommandJoystick;
import org.usfirst.frc4904.standard.custom.controllers.CustomCommandXbox;
import org.usfirst.frc4904.standard.custom.motorcontrollers.CANTalonFX;
import org.usfirst.frc4904.standard.custom.motorcontrollers.CustomCANSparkMax;
import org.usfirst.frc4904.standard.custom.motorcontrollers.SmartMotorController;

// import org.usfirst.frc4904.standard.LogKitten;

// import org.usfirst.frc4904.standard.custom.motorcontrollers.CustomCANSparkMax;
//import org.usfirst.frc4904.standard.subsystems.motor.SparkMaxMotorSubsystem;

public class RobotMap {

    public static class Port {

        public static class HumanInput {

            public static final int xyJoystickPort = 0;
            public static final int zJoystickPort = 1;

            public static final int joystick = 2;
        }

        public static class CANMotor {

            public static final int RAMP = 10;

            public static final int OUTTAKE_MOTOR_RIGHT = 26;
            public static final int OUTTAKE_MOTOR_LEFT = 27;

            public static final int CLIMBER = 60;

            public static final int ELEVATOR_RIGHT = 15;
            public static final int ELEVATOR_LEFT = 16;
        }

        public static class PWM {

            public static final int ENCODER_FL = 0;
            public static final int ENCODER_FR = 1;
            public static final int ENCODER_BL = 2;
            public static final int ENCODER_BR = 3;

            public static final int ELEVATOR_ENCODER = 0;

            public static final int LED_STRIP = 9;
        }
    }

    public static class Component {

        public static CANTalonFX flDrive;
        public static CANTalonFX frDrive;
        public static CANTalonFX blDrive;
        public static CANTalonFX brDrive;

        public static CustomEncoder elevatorEncoder;

        public static AHRS navx;

        // subsystems
        public static SwerveSubsystem chassis;
        public static MotorSubsystem ramp;
        public static ElevatorSubsystem elevator;
        public static MotorSubsystem outtake;
        public static MotorSubsystem climber;
        

        // motors
        public static CustomCANSparkMax rampMotor;

        public static CustomCANSparkMax outtakeMotorLeft;
        public static CustomCANSparkMax outtakeMotorRight;

        public static CANTalonFX elevatorMotorOne;
        public static CANTalonFX elevatorMotorTwo;

        public static CANTalonFX climberMotor;

        public static AddressableLED ledStrip;
    }

    public static class NetworkTables {

        public static NetworkTableInstance instance;

        public static class Odometry {

            public static NetworkTable table;
            public static NetworkTableEntry pose;
            public static NetworkTableEntry accel;
            public static NetworkTableEntry turretAngle;
        }

        public static class Localization {

            public static NetworkTable table;
            public static NetworkTableEntry goalDistance;
            public static NetworkTableEntry goalRelativeAngle;
        }
    }

    public static class Input {}

    public static class HumanInput {

        public static class Driver {

            public static CustomCommandXbox xbox;
            public static CustomCommandJoystick xyJoystick;
            public static CustomCommandJoystick turnJoystick;
        }

        public static class Operator {

            public static CustomCommandJoystick joystick;
        }
    }

    public RobotMap() {
        Component.navx = new AHRS(NavXComType.kMXP_SPI);

        var flTurn = new CustomCANSparkMax(5, MotorType.kBrushless, false);
        var frTurn = new CustomCANSparkMax(6, MotorType.kBrushless, false);
        var blTurn = new CustomCANSparkMax(7, MotorType.kBrushless, false);
        var brTurn = new CustomCANSparkMax(8, MotorType.kBrushless, false);

        Component.chassis = new SwerveSubsystem(
            new SwerveModule(
                new CANTalonFX(1),
                flTurn,
                flTurn.getAbsoluteEncoder(),
                new Translation2d(-1, 1)
            ),
            new SwerveModule(
                new CANTalonFX(2),
                frTurn,
                frTurn.getAbsoluteEncoder(),
                new Translation2d(1, 1)
            ),
            new SwerveModule(
                new CANTalonFX(3),
                blTurn,
                blTurn.getAbsoluteEncoder(),
                new Translation2d(-1, -1)
            ),
            new SwerveModule(
                new CANTalonFX(4),
                brTurn,
                brTurn.getAbsoluteEncoder(),
                new Translation2d(1, -1)
            )
        );

        // Component.chassis.swerveDrive.setGyroOffset(new Rotation3d(0, 0, Units.degreesToRadians(180)));


        Component.rampMotor = new CustomCANSparkMax(
            Port.CANMotor.RAMP,
            SparkLowLevel.MotorType.kBrushless,
            false
        );
        Component.ramp = new MotorSubsystem(Component.rampMotor, -7);

        Component.outtakeMotorLeft = new CustomCANSparkMax(
            Port.CANMotor.OUTTAKE_MOTOR_LEFT,
            SparkLowLevel.MotorType.kBrushless,
            false
        );
        Component.outtakeMotorRight = new CustomCANSparkMax(
            Port.CANMotor.OUTTAKE_MOTOR_RIGHT,
            SparkLowLevel.MotorType.kBrushless,
            false
        );
        Component.outtake = new MotorSubsystem(
            new SmartMotorController[] { Component.outtakeMotorLeft, Component.outtakeMotorRight },
            -4
        );

        Component.climberMotor = new CANTalonFX(Port.CANMotor.CLIMBER);
        Component.climber = new MotorSubsystem(Component.climberMotor, 6);

        Component.elevatorMotorOne = new CANTalonFX(Port.CANMotor.ELEVATOR_LEFT);
        Component.elevatorMotorTwo = new CANTalonFX(Port.CANMotor.ELEVATOR_RIGHT);
        Component.elevatorEncoder = new CustomEncoder(Port.PWM.ELEVATOR_ENCODER);
        Component.elevator = new ElevatorSubsystem(
            Component.elevatorMotorOne,
            Component.elevatorMotorTwo,
            Component.elevatorEncoder
        );

    

        HumanInput.Driver.xyJoystick = new CustomCommandJoystick(
            Port.HumanInput.xyJoystickPort,
            0.0
        );
        HumanInput.Driver.turnJoystick = new CustomCommandJoystick(
            Port.HumanInput.zJoystickPort,
            0.0
        );

        HumanInput.Operator.joystick = new CustomCommandJoystick(Port.HumanInput.joystick, 0.01);
    }
}
