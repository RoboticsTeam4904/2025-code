package org.usfirst.frc4904.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
//import com.ctre.phoenix.motorcontrol.InvertType; //broken
//import com.revrobotics.CANSparkMax.IdleMode; //broken
import com.revrobotics.spark.SparkLowLevel.MotorType;
//imports for rev robotics neo 550s
import com.revrobotics.spark.SparkMax;
import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.SerialPort;
import java.io.File;
import org.usfirst.frc4904.robot.subsystems.SwerveSubsystem;
import org.usfirst.frc4904.standard.custom.controllers.CustomCommandJoystick;
import org.usfirst.frc4904.standard.custom.controllers.CustomCommandXbox;
import org.usfirst.frc4904.standard.custom.motorcontrollers.CANTalonFX;
// import org.usfirst.frc4904.standard.LogKitten;

//import org.usfirst.frc4904.standard.custom.motorcontrollers.CustomCANSparkMax;
//import org.usfirst.frc4904.standard.subsystems.motor.SparkMaxMotorSubsystem;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class RobotMap {

    public static class Port {

        public static class HumanInput {

            public static final int xbox = 1;
            public static final int joystick = 2;
        }

        // 2023 robot constants // TODO: update ports for swerve
        public static class CANMotor {

            public static final int FRONT_LEFT_DRIVE = 1;
            public static final int FRONT_LEFT_TURN = 5;
            public static final int FRONT_RIGHT_DRIVE = 2;
            public static final int FRONT_RIGHT_TURN = 6;
            public static final int BACK_LEFT_DRIVE = 3;
            public static final int BACK_LEFT_TURN = 7;
            public static final int BACK_RIGHT_DRIVE = 4;
            public static final int BACK_RIGHT_TURN = 8;
        }

        public static class PWM {

            public static final int ENCODER_FL = 0;
            public static final int ENCODER_FR = 1;
            public static final int ENCODER_BL = 2;
            public static final int ENCODER_BR = 3;
        }

        public static class CAN {}

        public static class Pneumatics {}

        public static class Digital {}
    }

    public static class Metrics {

        // // 2023-robot constants
        public static class Chassis {

            public static final double GEAR_RATIO_DRIVE = 5.1; // 5.1:1 gear ratio
            public static final double GEAR_RATIO_TURN = 46.42; // 46.42:1 gear ratio
            public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(3); // 3 inch wheels
            public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(12.241 * 2); // +/- 0.5 inches
            public static final double TRACK_LENGTH_METERS = Units.inchesToMeters(12.259 * 2); //
            public static final double CHASSIS_LENGTH = Units.inchesToMeters(28); // +/- 0.5 inches
            public static final Translation2d CENTER_MASS_OFFSET = new Translation2d(0, 0); // no offset
            public static final double EncoderTicksPerRevolution = 2048;

            //these are allowed maxes rather than max capabilities
            //constants taken from https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/main/src/main/java/frc/robot/Constants.java
            public static final double MAX_SPEED = 4.8; //allowed max speed in meters per second
            public static final double MAX_ACCELERATION = 3; //allowed max acceleration in meters per second squared
            public static final double MAX_TRANSLATION_SPEED = 4.8;
            public static final double MAX_TURN_SPEED = 360; //allowed max turn speed in degrees per second
            public static final double MAX_TURN_ACCELERATION = 180; //allowed max turn acceleration in degrees per second squared
        }
    }

    public static class PID {

        public static class Drive {

            // PID constants
            // public static final double kP = 1.5;
            public static final double kP = .04; //TODO: tune, this is from maxswerve repo but seems too low
            public static final double kI = 0; // FIXME: tune
            public static final double kD = 0;
            // feedforward constants
            // pre-sfr on-carpet characterization
            // public static final double kS = 0.025236;
            // public static final double kV = 3.0683;
            // public static final double kA = 0.7358;
            //post sfr characterization
            public static final double kS = .02; //TODO: these are placeholders use sysid to find these
            public static final double kV = 3;
            public static final double kA = .5;
        }

        public static class Turn { //TODO: tune

            // PID constants
            public static final double kP = 1;
            public static final double kI = 0;
            public static final double kD = 0;
            // feedforward constants
            public static final double kS = .02; //TODO:these are placeholders use sysid to find these
            public static final double kV = 3;
            public static final double kA = .5;
        }
    }

    public static class Component {

        //TODO: turn motors are NOT falcons and so can't use cantalons
        public static CANTalonFX FLdrive;
        // public static CustomCANSparkMax FLturn;
        public static CANTalonFX FRdrive;
        // public static CustomCANSparkMax FRturn;
        public static CANTalonFX BLdrive;
        // public static CustomCANSparkMax BLturn;
        public static CANTalonFX BRdrive;
        // public static CustomCANSparkMax BRturn;

        //encoders are dutycycle encoders, not standard can encoders
        public static DutyCycleEncoder FLturnEncoder;
        public static DutyCycleEncoder FRturnEncoder;
        public static DutyCycleEncoder BLturnEncoder;
        public static DutyCycleEncoder BRturnEncoder;

        public static AHRS navx;

        // public static RobotUDP robotUDP;
        //Subsystems
        public static SwerveSubsystem chassis;
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
        }

        public static class Operator {

            public static CustomCommandJoystick joystick;
        }
    }

    public RobotMap() {
        Component.chassis = new SwerveSubsystem(
            new File(Filesystem.getDeployDirectory(), "swerve"),
            360,
            0.0473,
            4.5
        );
        // Component.navx = new AHRS(SerialPort.Port.kMXP);

        HumanInput.Driver.xbox = new CustomCommandXbox(Port.HumanInput.xbox, 0.01);

        // HumanInput.Operator.joystick = new CustomCommandJoystick(Port.HumanInput.joystick, 0.01);
        // // // UDP things
        // // try {
        // //     Component.robotUDP = new RobotUDP(Port.Network.LOCAL_SOCKET_ADDRESS, Port.Network.LOCALIZATION_ADDRESS);
        // // } catch (IOException ex) {
        // //     LogKitten.f("Failed to initialize UDP subsystem");
        // //     LogKitten.ex(ex);
        // // }

        // /***********************
        //  * Chassis Subsystem
        // *************************/

        // //TODO: fix invert type, talk to anna

        Component.FLdrive = new CANTalonFX(Port.CANMotor.FRONT_LEFT_DRIVE);
        // Component.FLturn = new CustomCANSparkMax(Port.CANMotor.FRONT_LEFT_TURN, MotorType.kBrushless, false);
        Component.FRdrive = new CANTalonFX(Port.CANMotor.FRONT_RIGHT_DRIVE);
        // Component.FRturn = new CustomCANSparkMax(Port.CANMotor.FRONT_RIGHT_TURN, MotorType.kBrushless, false);
        Component.BLdrive = new CANTalonFX(Port.CANMotor.BACK_LEFT_DRIVE);
        // Component.BLturn = new CustomCANSparkMax(Port.CANMotor.BACK_LEFT_TURN, MotorType.kBrushless, false);
        Component.BRdrive = new CANTalonFX(Port.CANMotor.BACK_RIGHT_DRIVE);
        // Component.BRturn = new CustomCANSparkMax(Port.CANMotor.BACK_RIGHT_TURN, MotorType.kBrushless, false);

        // // Component.backRightWheelTalon.setSafetyEnabled(false);
        // // Component.frontRightWheelTalon.setSafetyEnabled(false);
        // // Component.backLeftWheelTalon.setSafetyEnabled(false);
        // // Component.frontLeftWheelTalon.setSafetyEnabled(false);

        // //TalonMotorSubsystem rightDriveMotors = new TalonMotorSubsystem("right drive motors", NeutralMode.Brake, 0, Component.frontRightWheelTalon, Component.backRightWheelTalon);
        // //FR is ++, FL is +-, BR is -+, BL is --
        // Translation2d locationFL = new Translation2d(Metrics.Chassis.TRACK_LENGTH_METERS / 2, -(Metrics.Chassis.TRACK_WIDTH_METERS / 2));
        // Translation2d locationFR = new Translation2d(Metrics.Chassis.TRACK_LENGTH_METERS / 2, Metrics.Chassis.TRACK_WIDTH_METERS / 2);
        // Translation2d locationBL = new Translation2d(-(Metrics.Chassis.TRACK_LENGTH_METERS / 2), -(Metrics.Chassis.TRACK_WIDTH_METERS / 2));
        // Translation2d locationBR = new Translation2d(-(Metrics.Chassis.TRACK_LENGTH_METERS / 2), Metrics.Chassis.TRACK_WIDTH_METERS / 2);
        // SwerveDriveKinematics kinematics = new SwerveDriveKinematics(locationFL, locationFR, locationBL, locationBR);

        // Component.FLturnEncoder = new DutyCycleEncoder(Port.PWM.ENCODER_FL); //TODO: fix port
        // Component.FRturnEncoder = new DutyCycleEncoder(Port.PWM.ENCODER_FR); //TODO: fix port
        // Component.BLturnEncoder = new DutyCycleEncoder(Port.PWM.ENCODER_BL); //TODO: fix port
        // Component.BRturnEncoder = new DutyCycleEncoder(Port.PWM.ENCODER_BR); //TODO: fix port
        // Component.FLturnEncoder.setPositionOffset(.45); //TODO: fix offset
        // Component.FRturnEncoder.setPositionOffset(.037); //TODO: fix offset
        // Component.BLturnEncoder.setPositionOffset(.7344); //TODO: fix offset
        // Component.BRturnEncoder.setPositionOffset(.651); //TODO: fix offset

        // Component.FLmodule  = new SwerveModule(Component.FLdrive, Component.FLturn, Component.FLturnEncoder, locationFL, "FLmodule");
        // Component.FRmodule = new SwerveModule(Component.FRdrive, Component.FRturn, Component.FRturnEncoder, locationFR, "FRmodule");
        // Component.BLmodule   = new SwerveModule(Component.BLdrive, Component.BLturn, Component.BLturnEncoder, locationBL, "BLmodule");
        // Component.BRmodule  = new SwerveModule(Component.BRdrive, Component.BRturn, Component.BRturnEncoder, locationBR, "BRmodule");
        // SwerveModule[] modules = {Component.FLmodule, Component.FRmodule, Component.BLmodule, Component.BRmodule};

        // //SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(getHeading()));
        // Component.chassis = new SwerveDrive(modules, kinematics, Component.navx, Metrics.Chassis.CENTER_MASS_OFFSET, new Pose2d(0,0,new Rotation2d(0)));

        // // Autonomous.autonCommand = Component.chassis.c_buildPathPlannerAuto(
        // //     PID.Drive.kS, PID.Drive.kV, PID.Drive.kA,
        // //     Autonomous.RAMSETE_B, Autonomous.RAMSETE_ZETA,
        // //     Autonomous.AUTON_NAME, Autonomous.MAX_VEL, Autonomous.MAX_ACCEL,
        // //     Autonomous.autonEventMap
        // // );

    }
}
