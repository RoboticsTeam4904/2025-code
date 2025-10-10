package org.usfirst.frc4904.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.usfirst.frc4904.robot.RobotMap.Component;
import org.usfirst.frc4904.standard.Util;
import org.usfirst.frc4904.standard.commands.WaitWhileCommand;

import java.util.*;

/** Sponsored by Claude™ 3.7 Sonnet by Anthropic® */
public class VisionSubsystem extends SubsystemBase {
    public enum TagGroup {
        ANY,
        INTAKE,
        REEF,
        REEF_INNER_CENTER,
        REEF_OUTER_CENTER,
        REEF_INNER_DIAGONAL,
        REEF_OUTER_DIAGONAL
    }

    private static final int TAGS_PER_FIELD_SIDE = 11;
    private static final HashMap<TagGroup, int[]> tagIds = new HashMap<>();

    static {
        // -1 means any tag (needs to be the first item in the array)
        tagIds.put(TagGroup.ANY, new int[] { -1 });
        tagIds.put(TagGroup.INTAKE, new int[] { 1, 2 });
        tagIds.put(TagGroup.REEF, new int[] { 6, 7, 8, 9, 10, 11 });
        tagIds.put(TagGroup.REEF_INNER_CENTER, new int[] { 10 });
        tagIds.put(TagGroup.REEF_OUTER_CENTER, new int[] { 7 });
        tagIds.put(TagGroup.REEF_INNER_DIAGONAL, new int[] { 9, 11 });
        tagIds.put(TagGroup.REEF_OUTER_DIAGONAL, new int[] { 6, 8 });

        // move april tags to other side of board if on the blue alliance
        if (DriverStation.getAlliance().orElse(null) == DriverStation.Alliance.Blue) {
            for (var ids : tagIds.values()) {
                if (ids[0] == -1) continue; // skip ANY

                for (int i = 0; i < ids.length; i++) {
                    ids[i] += TAGS_PER_FIELD_SIDE;
                }
            }
        }
    }

    // sideways distance off of the april tag to align to
    private static final double HORIZ_ALIGN_OFFSET = Units.inchesToMeters(6.5);

    // max speeds
    private final double MAX_LINEAR_SPEED = 5; // meters per second
    private final double MAX_ROT_SPEED = 2 * Math.PI; // radians per second

    // tolerance thresholds for positioning
    private final double POS_TOLERANCE_METERS = 0.005;
    private final double ROT_TOLERANCE_DEG = 1.0;

    // all timeouts are in seconds
    private final double CANT_SEE_TIMEOUT = 1; // give up if we cant see the april tag for this many seconds
    private final double TOTAL_TIMEOUT = 5; // always give up after this many seconds

    private record CameraTag(PhotonTrackedTarget tag, int cameraIndex) {}

    private final PhotonCamera[] photonCameras;

    // pid controllers
    private final PIDController positionController;
    private final PIDController rotationController;

    // target pose relative to tag
    private Transform2d offset;

    private double startTime = 0;
    private double lastSeenTagTime = 0;

    // ids for all of the possible tags to target
    private int[] targetTagOptions = null;

    // int if aligning to a certain tag, null if not
    private Integer targetTagId = null;
    private Transform2d desiredOffset = null;

    // used to estimate position when we can't see the tag
    private double lastTime;
    private ChassisSpeeds lastSpeed = null;

    // camera positions relative to robot center
    private final Transform2d[] cameraOffsets;

    /**
     * Creates a new VisionSubsystem
     *
     * @param photonCameras The PhotonVision cameras
     * @param cameraOffsets The transforms from the camera to the robot center.
     *                      -X is towards the front of the robot.
     */
    public VisionSubsystem(PhotonCamera[] photonCameras, Transform2d[] cameraOffsets) {
        this.photonCameras = photonCameras;
        this.cameraOffsets = cameraOffsets;

        // initialize pid controllers
        // TODO TUNING: tune pid values
        positionController = new PIDController(1, 0, 0);
        rotationController = new PIDController(1, 0, 0);

        // make rotation controller continuous
        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        // set tolerances
        positionController.setTolerance(POS_TOLERANCE_METERS);
        rotationController.setTolerance(Math.toRadians(ROT_TOLERANCE_DEG));
    }

    private double startingDistance = -1;
    private double startingRotDistance = -1;

    @Override
    public void periodic() {
        if (!this.isPositioning()) return;

        double currentTime = Timer.getFPGATimestamp();

        CameraTag target;

        if (targetTagId == null) {
            // find best tag out of possible tags and target it
            target = getBestTargetId(targetTagOptions);
            targetTagId = target != null ? target.tag.fiducialId : null;
        } else {
            target = getTarget(targetTagId);
        }

        if (target == null) {
            // stop positioning if tag has not been seen for a while
            double timeElapsed = currentTime - lastSeenTagTime;
            if (timeElapsed > CANT_SEE_TIMEOUT) {
                stopPositioning("No april tag visible for " + CANT_SEE_TIMEOUT + " seconds", true);
                return;
            }

            if (lastSpeed == null || desiredOffset == null) return;

            double deltaTime = currentTime - lastTime;
            desiredOffset = desiredOffset.plus(
                new Transform2d(
                    -lastSpeed.vxMetersPerSecond * deltaTime,
                    -lastSpeed.vyMetersPerSecond * deltaTime,
                    new Rotation2d(-lastSpeed.omegaRadiansPerSecond * deltaTime)
                )
            );
        } else {
            lastSeenTagTime = currentTime;

            // calculate position error relative to desired position
            desiredOffset = calculatePositionError(target);
        }

        // use pid to calculate needed speeds for x, y, rotation
        double xSpeed = positionController.calculate(0, desiredOffset.getX());
        double ySpeed = positionController.calculate(0, desiredOffset.getY());
        double rotSpeed = rotationController.calculate(0, desiredOffset.getRotation().getRadians());

        xSpeed = Util.clamp(xSpeed, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);
        ySpeed = Util.clamp(ySpeed, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED);
        rotSpeed = Util.clamp(rotSpeed, -MAX_ROT_SPEED, MAX_ROT_SPEED);

        // convert to robot relative speeds
        ChassisSpeeds relativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            xSpeed,
            ySpeed,
            rotSpeed,
            Rotation2d.kZero
        );

        // command swerve drive
        Component.chassis.drive(relativeSpeeds);

        lastTime = currentTime;
        lastSpeed = relativeSpeeds;

        // log positioning data
        System.out.printf(
            "Positioning to tag %s. X: %.4f, Y: %.4f, Rot: %.2fdeg%n",
            targetTagId,
            desiredOffset.getX(),
            desiredOffset.getY(),
            desiredOffset.getRotation().getDegrees()
        );

        // check if reached target position
        double distance = Math.hypot(desiredOffset.getX(), desiredOffset.getY());
        double rotDistance = Math.abs(desiredOffset.getRotation().getDegrees());

        if (startingDistance == -1) startingDistance = distance;
        if (startingRotDistance == -1) startingRotDistance = rotDistance;

        Component.lights.visionProgress = Math.pow(
            (1 - distance / startingDistance) * 0.6 + (1 - rotDistance / startingRotDistance) * 0.4,
            2
        );

        boolean atPosition = distance < POS_TOLERANCE_METERS && rotDistance < ROT_TOLERANCE_DEG;

        if (atPosition) {
            stopPositioning("Success", false);
            Component.lights.flashColor(LightSubsystem.Color.SUCCESS);
        } else {
            // give up if too much time has passed
            double timeElapsed = currentTime - startTime;
            if (timeElapsed > TOTAL_TIMEOUT) {
                stopPositioning("Gave up after " + TOTAL_TIMEOUT + " seconds", true);
            }
        }
    }

    /**
     * Find the best April Tag to target given a list of April Tag IDs
     *
     * @param tagIds The IDs of the April Tags to target
     * @return The ID of the best April Tag, or null if none were found
     */
    private CameraTag getBestTargetId(int[] tagIds) {
        List<CameraTag> results = getResults();

        if (results.isEmpty()) return null;

        // -1 represents any, so return any best result
        if (tagIds[0] == -1) {
            return results.get(0);
        }

        // find best tag in the possible tag options
        for (var target : results) {
            for (int tagId : tagIds) {
                if (tagId == target.tag.fiducialId) {
                    return target;
                }
            }
        }

        return null;
    }

    /**
     * Get a PhotonVision target for an April Tag matching a certain ID
     *
     * @param tagId The ID of the tag to look for
     * @return A {@link CameraTag} or {@code null} if no April Tag was found
     */
    CameraTag getTarget(int tagId) {
        for (var target : getResults()) {
            if (tagId == target.tag.fiducialId) {
                return target;
            }
        }

        return null;
    }

    /**
     * @return A list of possible targets
     */
    List<CameraTag> getResults() {
        List<CameraTag> results = new ArrayList<>();

        for (int i = 0; i < photonCameras.length; i++) {
            List<PhotonPipelineResult> unreadResults = photonCameras[i].getAllUnreadResults();
            if (unreadResults.isEmpty()) continue;

            double lastCaptureTime = 0;
            for (var result : unreadResults) {
                lastCaptureTime = Math.max(lastCaptureTime, result.getTimestampSeconds());
            }

            for (var result : unreadResults) {
                // discard results that are more than 0.1 seconds older than the latest result
                if (lastCaptureTime - result.getTimestampSeconds() > 0.1) continue;

                for (var target : result.getTargets()) {
                    results.add(new CameraTag(target, i));
                }
            }
        }

        results.sort(Comparator.comparingDouble(result -> {
            Transform3d transform = result.tag.getBestCameraToTarget();
            return Math.pow(transform.getX(), 2) + Math.pow(transform.getY(), 2);
        }));

        return results;
    }

    private void startPositioning(int[] targetTagIds, Transform2d offset) {
        this.targetTagOptions = targetTagIds;
        this.offset = offset;

        System.out.println("VISION OFFSET 2: X " + this.offset.getX() + " Y " + this.offset.getY());

        startTime = lastSeenTagTime = lastTime = Timer.getFPGATimestamp();

        // reset pid controllers
        positionController.reset();
        rotationController.reset();

        System.out.println("Positioning started");

        Component.lights.visionProgress = 0;
        Component.lights.flashColor(LightSubsystem.Color.VISION);
    }

    /**
     * Stop the positioning process
     */
    public void stopPositioning() {
        stopPositioning(null, false);
    }

    /**
     * Stop the positioning process
     *
     * @param reason The reason that positioning was stopped (for logging), e.g. "Success"
     * @param failed Whether positioning was stopped due to some sort of failure. Only for the purpose of fancy lights.
     */
    public void stopPositioning(String reason, boolean failed) {
        targetTagOptions = null;
        targetTagId = null;
        desiredOffset = null;
        Component.chassis.drive(new ChassisSpeeds(0, 0, 0));

        System.out.println("Positioning ended" + (reason != null ? " - " + reason : ""));

        Component.lights.visionProgress = -1;
        if (failed) {
            Component.lights.flashColor(LightSubsystem.Color.FAIL);
        }
    }

    /**
     * Calculate the position error between current position and desired position
     *
     * @param target The target to calculate alignment to
     * @return The transform from current position to desired position
     */
    private Transform2d calculatePositionError(CameraTag target) {
        Transform3d rawOffset = target.tag.getBestCameraToTarget();

        Transform2d targetOffset = new Transform2d(
            rawOffset.getX(),
            rawOffset.getY(),
            rawOffset.getRotation().toRotation2d()
        );

        Transform2d cameraOffset = cameraOffsets[target.cameraIndex];

        // calculate transform from robot to the tag
        Transform2d robotToTarget = new Transform2d(
            targetOffset.getTranslation().plus(cameraOffset.getTranslation().rotateBy(targetOffset.getRotation())),
            targetOffset.getRotation().plus(cameraOffset.getRotation())
        );

        System.out.println("VISION OFFSET 3: X " + offset.getX() + " Y " + offset.getY());

        // calculate difference between current and desired
        Translation2d translationError = offset.getTranslation().minus(robotToTarget.getTranslation());
        Rotation2d rotationError = offset.getRotation().minus(robotToTarget.getRotation());

        return new Transform2d(translationError, rotationError);
    }

    /**
     * Check if the subsystem is currently attempting to position
     *
     * @return true if positioning is in progress
     */
    public boolean isPositioning() {
        return targetTagOptions != null;
    }

    /**
     * Start aligning to an April Tag that matches the ID given
     *
     * @param targetTagId      The ID of the April Tag to align to
     * @param offsetMultiplier The horizontal offset multiplier from the tag.
     *                         For 2025 Reefscape: -1 is left coral, 0 is center, 1 is right coral.
     */
    public Command c_align(int targetTagId, int offsetMultiplier) {
        return c_align(new int[] { targetTagId }, offsetMultiplier);
    }

    /**
     * Start aligning to an April Tag that matches the {@link TagGroup} given
     *
     * @param targetTagGroup   A group of april tags to align to, e.g. {@code TagGroup.REEF}.
     *                         The robot will align to whichever one PhotonVision considers the best
     * @param offsetMultiplier The horizontal offset multiplier from the tag.
     *                         For 2025 Reefscape: -1 is left coral, 0 is center, 1 is right coral.
     */
    public Command c_align(TagGroup targetTagGroup, int offsetMultiplier) {
        return c_align(tagIds.get(targetTagGroup), offsetMultiplier);
    }

    /**
     * Start aligning to an April Tag that matches one of the IDs given
     *
     * @param targetTagIds     A list of april tag IDs to align to.
     *                         The robot will align to whichever one PhotonVision considers the best
     * @param offsetMultiplier The horizontal offset multiplier from the tag.
     *                         For 2025 Reefscape: -1 is left coral, 0 is center, 1 is right coral.
     */
    public Command c_align(int[] targetTagIds, int offsetMultiplier) {
        Transform2d offset = new Transform2d(0, HORIZ_ALIGN_OFFSET * offsetMultiplier, Rotation2d.kPi);

        // System.out.println("VISION OFFSET 1: X " + offset.getX() + " Y " + offset.getY());

        var command = new SequentialCommandGroup(
            this.runOnce(() -> startPositioning(targetTagIds, offset)),
            new WaitWhileCommand(this::isPositioning)
        ) {
            @Override
            public void cancel() {
                super.cancel();
                stopPositioning("Command canceled", false);
            }

            @Override
            public InterruptionBehavior getInterruptionBehavior() {
                return InterruptionBehavior.kCancelIncoming;
            }
        };
        command.addRequirements(Component.chassis);
        return command;
    }

    public Command c_stop() {
        return new InstantCommand(() -> this.stopPositioning("Stop command", false));
    }
}
