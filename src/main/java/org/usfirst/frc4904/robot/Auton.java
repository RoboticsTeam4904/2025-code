package org.usfirst.frc4904.robot;

import edu.wpi.first.wpilibj2.command.*;
import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.robot.subsystems.ElevatorSubsystem;
import org.usfirst.frc4904.robot.vision.VisionSubsystem.TagGroup;

public class Auton {

    public static boolean getFlipSide() {
        return Robot.AutonConfig.FLIP_SIDE;
    }

    // public static Command c_jank() {
    //     return new ParallelDeadlineGroup(
    //         new WaitCommand(1),
    //         new InstantCommand(() -> RobotMap.Component.chassis.drive(
    //             ChassisSpeeds.fromRobotRelativeSpeeds(
    //                 1.0,
    //                 0.0,
    //                 0.0,
    //                 Rotation2d.kZero
    //             )
    //         ))
    //     );
    // }

    /**
     * Move straight out of the starting zone and do nothing.
     */
    public static Command c_straight() {
        return Component.chassis.getAutonomousCommand("straight", true, false);
    }

     /**
     * Move straight out of the starting zone, align with the center of the reef, and outtake.
     * PUTS THE CORAL ON THE ALGAE :skull:
     */
    public static Command c_jankStraightCoral() {
        return new SequentialCommandGroup(
            new WaitCommand(5),
            Component.chassis.c_driveRobotRelative(0.4, 0, 0).withTimeout(2),
            Component.chassis.c_stop(),
            Component.vision.c_align(new int[] { 10, 21 }, -1),
            Component.elevator.c_outtakeAtPosition(ElevatorSubsystem.Position.L3),
            Component.elevator.c_gotoPosition(ElevatorSubsystem.Position.INTAKE)
        );
    }

    // actually moves backwards - robot must be placed physically backwards on the field
    public static Command c_jankStraight() {
        return new SequentialCommandGroup(
            new WaitCommand(12),
            Component.chassis.c_driveRobotRelative(-0.5, 0, 0).withTimeout(2),
            Component.chassis.c_stop()
        );
    }

    // actually moves forwards
    public static Command c_jankReverse() {
        return new SequentialCommandGroup(
            new WaitCommand(12),
            Component.chassis.c_driveRobotRelative(0.5, 0, 0).withTimeout(2),
            Component.chassis.c_stop()
        );
    }

    public static Command c_jankLeftCoral() {
        return c_jankSideCoral(1);
    }

    public static Command c_jankRightCoral() {
        return c_jankSideCoral(-1);
    }

    /** @param side 1 = robot-relative left, -1 = robot-relative right */
    private static Command c_jankSideCoral(int side) {
        return new SequentialCommandGroup(
            new WaitCommand(3),
            Component.chassis.c_driveRobotRelative(-0.4 / Math.sqrt(3), -0.4 * side, 0).withTimeout(1.5),
            Component.chassis.c_stop(),
            Component.vision.c_align(new int[] { 9, 11, 20, 22 }, -1),
            Component.elevator.c_outtakeAtPosition(ElevatorSubsystem.Position.L2),
            Component.elevator.c_gotoPosition(ElevatorSubsystem.Position.INTAKE)
        );
    }

    // /**
    //  * Move straight out of the starting zone, align with the center of the reef, and outtake.
    //  */
    // public static Command c_straightCoral() {
    //     return new SequentialCommandGroup(
    //         // Component.chassis.getAutonomousCommand("straight", true, getFlipAlliance(), false),
    //         Component.vision.c_align(TagGroup.REEF_INNER_CENTER, -1),
    //         Component.elevator.c_outtakeAtPosition(ElevatorSubsystem.Position.L2)
    //     );
    // }

    /**
     * Move diagonally from the starting zone, align with the inner diagonal of the reef, and outtake.
     */
    public static Command c_sideCoral() {
        return new SequentialCommandGroup(
            Component.chassis.getAutonomousCommand("side", true, getFlipSide()),
            Component.vision.c_align(TagGroup.REEF_INNER_DIAGONAL, -1)
            //Component.elevator.c_outtakeAtPosition(ElevatorSubsystem.Position.L2)
        );
    }

    /**
     * <ol>
     *   <li> Move diagonally from the starting zone
     *   <li> Align with the inner diagonal of the reef
     *   <li> Outtake
     *   <li> Move to and align with the coral intake station
     *   <li> Wait 2 seconds for intake
     *   <li> Move to and align with the outer diagonal of the reef
     *   <li> Outtake
     * </ol>
     */
    public static Command c_sideCoralFancy() {
        return new SequentialCommandGroup(
            c_sideCoral(),
            Component.chassis.getAutonomousCommand("side2", true, getFlipSide()),
            new WaitCommand(2),
            //Component.elevator.c_intake(),
            Component.chassis.getAutonomousCommand("side3", true, getFlipSide()),
            Component.vision.c_align(TagGroup.REEF_OUTER_DIAGONAL, -1)
            //Component.elevator.c_outtakeAtPosition(ElevatorSubsystem.Position.L2)
        );
    }
}
