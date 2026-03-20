package frc.robot.subsystems.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.limelight.LimelightHelpers.RawFiducial;

/**
 * Manages all vision functions for the robot.
 *
 * Hardware:
 *   LL4 (rear, facing backward) — pose estimation via MegaTag2 using Pigeon2 IMU
 *   LL2 (front, facing forward) — pose estimation via MegaTag2 using external Pigeon2 IMU,
 *                                  AprilTag alignment tx, and shooter distance measurement
 *
 * Pose pipeline:
 *   Both limelights run MegaTag2 independently and contribute measurements to the
 *   drivetrain Kalman filter. The LL2 uses the Pigeon2 yaw sent via SetRobotOrientation()
 *   as its external IMU source — the LL2 does not have a built-in IMU but supports
 *   external IMU fusion for MegaTag2. Both cameras face different directions so they
 *   see different tags and complement each other throughout the match:
 *     - LL4 sees alliance wall and mid-field tags when driving away from the hub
 *     - LL2 sees Hub tags when approaching to score
 *   Each measurement is independently validated and weighted by distance and tag count
 *   via dynamic standard deviations before being fed to the Kalman filter.
 *
 * Alignment:
 *   calculateAimVelocity() reads tx from the LL2 so the robot aligns to hub
 *   tags directly in front of the robot rather than rear field tags.
 *
 * Distance:
 *   getBestDistanceToTarget() prefers LL2 distToCamera when a hub tag is visible
 *   and falls back to LL4 hub-tag-filtered distance. Used by the flywheel velocity model.
 *
 * LEDs:
 *   setRearLEDs*()  — controls LL4 LEDs, turned on during auto
 *   setFrontLEDs*() — controls LL2 LEDs, turned on during AprilTag alignment
 */
public class LimelightSubsystem extends SubsystemBase {

    // LL4 — rear-facing, primary pose estimation camera
    private static final String kLimelight1  = "limelight-one";

    // LL2 — front-facing, secondary pose + alignment + hub distance
    // Must match the hostname set in the LL2 web interface exactly
    private static final String kLimelight2 = "limelight-two";

    // Proportional gain for tx-based rotation alignment (right bumper command)
    // Tune on carpet: increase if robot doesn't rotate, decrease if it oscillates
    private static final double kPAim = 0.035;

    // -------------------------------------------------------------------------
    // Pose rejection thresholds — loosened for 2026 field limited tag visibility
    // -------------------------------------------------------------------------

    // Maximum ambiguity ratio for single-tag estimates (0 = perfect, 1 = ambiguous)
    private static final double kMaxAmbiguity         = 0.4;

    // Maximum distance for accepted tag measurements
    private static final double kMaxSingleTagDistance = 5.5;  // meters
    private static final double kMaxMultiTagDistance  = 7.0;  // meters

    // Maximum odometry-to-vision jump before measurement is rejected
    private static final double kMaxPoseJumpTeleop    = 1.5;  // meters
    private static final double kMaxPoseJumpAuto      = 1.0;  // meters

    // Maximum rotation speed — MegaTag2 yaw seed becomes stale when spinning fast
    private static final double kMaxRotationDegPerSec = 720.0; // deg/s

    // -------------------------------------------------------------------------
    // 2026 field dimensions (meters) — used for out-of-field pose rejection
    // -------------------------------------------------------------------------
    private static final double kFieldLengthMeters = 17.548;
    private static final double kFieldWidthMeters  = 8.211;
    private static final double kFieldMarginMeters = 0.5;

    // -------------------------------------------------------------------------
    // Standard deviation base values for the Kalman filter
    // Higher = less trust in vision. Scaled by distance^kDistanceExp.
    // -------------------------------------------------------------------------
    private static final double kSingleTagXyBase    = 0.5;
    private static final double kSingleTagThetaBase = 0.9;
    private static final double kMultiTagXyBase     = 0.2;
    private static final double kMultiTagThetaBase  = 0.4;
    private static final double kDistanceExp        = 2.0;

    // Frames to skip after a mode change before accepting corrections (~0.5s at 50Hz)
    private static final int kWarmupFrames = 25;

    // How long a pose stays confident after the last accepted vision measurement.
    // Generous because the 2026 field layout means cameras may see tags intermittently.
    private static final double kPoseConfidenceWindowSeconds = 2.0;

    // -------------------------------------------------------------------------
    // Auto start pose reset — accepts single tag given limited field visibility
    // -------------------------------------------------------------------------
    private static final int    kAutoResetMinTags  = 1;
    private static final double kAutoResetMaxDist  = 5.0;  // meters
    private static final double kAutoResetMaxAmbig = 0.25;

    // -------------------------------------------------------------------------
    // Hub tag IDs — LL4 fallback in getBestDistanceToTarget() only accepts these
    // so non-hub field tags never corrupt shooter velocity calculations.
    // LL2 has no filter since it faces the hub — any visible tag is relevant.
    // -------------------------------------------------------------------------
    private static final java.util.Set<Integer> kHubTagIds = new java.util.HashSet<>(
        java.util.Arrays.asList(2, 3, 4, 5, 8, 9, 10, 11, 18, 19, 20, 21, 24, 25, 26, 27)
    );

    // -------------------------------------------------------------------------
    // Runtime state
    // -------------------------------------------------------------------------
    private CommandSwerveDrivetrain m_drivetrain;
    private boolean m_visionUpdatesEnabled = true;
    private boolean m_isAutoMode           = false;

    // Rotation speed tracking — used to reject MegaTag2 during fast spins
    private double m_lastYawDegrees       = 0.0;
    private double m_lastTimestamp        = 0.0;
    private double m_rotDegPerSec         = 0.0;
    private int    m_frameCount           = 0;

    // Timestamp of last accepted vision measurement from either camera
    private double m_lastVisionUpdateTime = 0.0;

    // Cached LL4 estimate — fetched once per loop, reused for telemetry
    // LL2 estimate is not cached since it isn't needed for SmartDashboard display
    private LimelightHelpers.PoseEstimate m_latestMt2Rear  = null;

    // Field2d widget showing raw vision pose alongside odometry pose in Elastic
    private final Field2d m_visionField = new Field2d();

    public LimelightSubsystem() {
        setRearLEDsOff();
        setFrontLEDsOff();
        SmartDashboard.putData("Vision/Field", m_visionField);
    }

    // =========================================================================
    // Public setters — called from Robot.java
    // =========================================================================

    /** Sets the drivetrain reference required for MegaTag2 odometry fusion. */
    public void setDrivetrain(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
    }

    /** Enables or disables vision odometry updates without affecting telemetry. */
    public void setVisionUpdatesEnabled(boolean enabled) {
        m_visionUpdatesEnabled = enabled;
    }

    /**
     * Switches between auto and teleop vision thresholds and resets the warmup counter.
     * Turns LL4 rear LEDs on during auto for maximum tag detection range.
     */
    public void setAutoMode(boolean isAuto) {
        m_isAutoMode = isAuto;
        m_frameCount = 0;
        if (isAuto) {
            setRearLEDsOn();
        } else {
            setRearLEDsOff();
        }
    }

    // =========================================================================
    // LL4 getters — rear camera
    // =========================================================================

    /** Returns horizontal offset from LL4 crosshair to target in degrees. */
    public double getHorizontalOffset() {
        return LimelightHelpers.getTX(kLimelight1);
    }

    /** Returns vertical offset from LL4 crosshair to target in degrees. */
    public double getVerticalOffset() {
        return LimelightHelpers.getTY(kLimelight1);
    }

    /** Returns target area from LL4 as a percentage of the image (0-100%). */
    public double getTargetArea() {
        return LimelightHelpers.getTA(kLimelight1);
    }

    /** Returns true if the LL4 has a valid target lock. */
    public boolean hasValidTarget() {
        return LimelightHelpers.getTV(kLimelight1);
    }

    /**
     * Returns distance from the LL4 camera lens to the primary AprilTag in meters.
     *
     * @return distance in meters, or -1.0 if no tag is visible
     */
    public double getDistanceToTarget() {
        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(kLimelight1);
        if (fiducials.length == 0) return -1.0;
        return fiducials[0].distToCamera;
    }

    // =========================================================================
    // LL2 getters — front camera
    // =========================================================================

    /** Returns true if the LL2 has a valid target lock. */
    public boolean hasFrontTarget() {
        return LimelightHelpers.getTV(kLimelight2);
    }

    /**
     * Returns distance from the LL2 camera lens to the primary AprilTag in meters.
     *
     * @return distance in meters, or -1.0 if no tag is visible
     */
    public double getFrontDistanceToTarget() {
        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(kLimelight2);
        if (fiducials.length == 0) return -1.0;
        return fiducials[0].distToCamera;
    }

    // =========================================================================
    // Fused distance — prefers LL2, falls back to LL4 hub tags only
    // =========================================================================

    /**
     * Returns the best available distance to a hub AprilTag in meters.
     *
     * Preference order:
     *   1. LL2 — faces hub directly, any visible tag is a hub tag, no ID filter needed
     *   2. LL4 — fallback, only accepted if tag ID is in kHubTagIds to prevent
     *             non-hub field tags from corrupting velocity calculations
     *   3. 1.0 m hardcoded fallback if neither camera sees a valid hub tag
     *
     * Monitor Vision/UsingFrontLL on SmartDashboard to see which source is active.
     */
    public double getBestDistanceToTarget() {
        RawFiducial[] frontFiducials = LimelightHelpers.getRawFiducials(kLimelight2);
        if (frontFiducials.length > 0) {
            double d = frontFiducials[0].distToCamera;
            if (d > 0 && d <= 10.0 && !Double.isNaN(d)) return d;
        }

        RawFiducial[] rearFiducials = LimelightHelpers.getRawFiducials(kLimelight1);
        for (RawFiducial fiducial : rearFiducials) {
            if (kHubTagIds.contains(fiducial.id)) {
                double d = fiducial.distToCamera;
                if (d > 0 && d <= 10.0 && !Double.isNaN(d)) return d;
            }
        }

        return 1.0;
    }

    /**
     * Returns a rotational rate for aligning to the current target using LL2 tx.
     * Reads from the front camera so the robot aligns toward hub tags.
     *
     * @param maxAngularRate the robot's maximum angular rate in radians per second
     * @return rotational rate in radians per second, or 0 if LL2 has no target
     */
    public double calculateAimVelocity(double maxAngularRate) {
        if (!hasFrontTarget()) return 0.0;
        return -LimelightHelpers.getTX(kLimelight2) * kPAim * maxAngularRate;
    }

    /** Returns the latest cached LL4 MegaTag2 pose estimate. Updated once per loop. */
    public LimelightHelpers.PoseEstimate getMegaTag2Estimate() {
        return m_latestMt2Rear;
    }

    /**
     * Returns true if a vision measurement was accepted from either camera within
     * the last kPoseConfidenceWindowSeconds seconds.
     */
    public boolean isPoseConfident() {
        return Timer.getFPGATimestamp() - m_lastVisionUpdateTime < kPoseConfidenceWindowSeconds;
    }

    // =========================================================================
    // LED control
    // =========================================================================

    /** Turns on the LL4 (rear) LEDs. Called during auto for maximum detection range. */
    public void setRearLEDsOn()   { LimelightHelpers.setLEDMode_ForceOn(kLimelight1);   }

    /** Turns off the LL4 (rear) LEDs. */
    public void setRearLEDsOff()  { LimelightHelpers.setLEDMode_ForceOff(kLimelight1);  }

    /** Turns on the LL2 (front) LEDs. Called during AprilTag alignment. */
    public void setFrontLEDsOn()  { LimelightHelpers.setLEDMode_ForceOn(kLimelight2);  }

    /** Turns off the LL2 (front) LEDs. */
    public void setFrontLEDsOff() { LimelightHelpers.setLEDMode_ForceOff(kLimelight2); }

    /** Switches the LL4 pipeline by index. Use to swap between AprilTag and detector modes. */
    public void setPipeline(int pipeline) {
        LimelightHelpers.setPipelineIndex(kLimelight1, pipeline);
    }

    // =========================================================================
    // Private helpers
    // =========================================================================

    private void putStatus(String msg) {
        SmartDashboard.putString("Vision/Status", msg);
    }

    /**
     * Sends the current gyro yaw to both limelights and tracks rotation speed.
     * Both cameras require the yaw for MegaTag2 — LL4 uses its built-in IMU
     * fused with the external yaw; LL2 uses the external yaw as its sole IMU source.
     * Must be called before reading any MegaTag2 estimate each loop.
     */
    private void seedOrientation() {
        if (m_drivetrain == null) return;

        double yaw = m_drivetrain.getState().Pose.getRotation().getDegrees();
        double now = Timer.getFPGATimestamp();

        if (m_lastTimestamp > 0.0) {
            double dt = now - m_lastTimestamp;
            if (dt > 0.0) {
                double delta = yaw - m_lastYawDegrees;
                // Wrap to [-180, 180] to handle gyro rollover
                while (delta >  180.0) delta -= 360.0;
                while (delta < -180.0) delta += 360.0;
                m_rotDegPerSec = Math.abs(delta / dt);
            }
        }

        m_lastYawDegrees = yaw;
        m_lastTimestamp  = now;

        // Send yaw to both limelights before reading either estimate this loop
        LimelightHelpers.SetRobotOrientation(kLimelight1,  yaw, 0, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation(kLimelight2, yaw, 0, 0, 0, 0, 0);
    }

    /**
     * Returns true if the pose estimate passes all rejection criteria.
     * Logs the rejection reason to SmartDashboard via Vision/Status.
     */
    private boolean isValidVisionMeasurement(LimelightHelpers.PoseEstimate est) {
        if (est.tagCount == 0) {
            putStatus("No tags");
            return false;
        }

        if (m_frameCount < kWarmupFrames) {
            putStatus("Warmup " + m_frameCount + "/" + kWarmupFrames);
            return false;
        }

        if (m_rotDegPerSec > kMaxRotationDegPerSec) {
            putStatus("Spinning: " + String.format("%.0f", m_rotDegPerSec) + " deg/s");
            return false;
        }

        if (est.tagCount == 1) {
            if (est.rawFiducials.length > 0 && est.rawFiducials[0].ambiguity > kMaxAmbiguity) {
                putStatus("Ambiguity " + String.format("%.2f", est.rawFiducials[0].ambiguity));
                return false;
            }
            if (est.avgTagDist > kMaxSingleTagDistance) {
                putStatus("Single tag too far: " + String.format("%.1f", est.avgTagDist) + "m");
                return false;
            }
        } else if (est.avgTagDist > kMaxMultiTagDistance) {
            putStatus("Multi tag too far: " + String.format("%.1f", est.avgTagDist) + "m");
            return false;
        }

        double px = est.pose.getX();
        double py = est.pose.getY();

        if (px < -kFieldMarginMeters || px > kFieldLengthMeters + kFieldMarginMeters
                || py < -kFieldMarginMeters || py > kFieldWidthMeters + kFieldMarginMeters) {
            putStatus("Outside field (" + String.format("%.1f,%.1f", px, py) + ")");
            return false;
        }

        if (px == 0.0 && py == 0.0) {
            putStatus("At origin");
            return false;
        }

        if (m_drivetrain != null) {
            Pose2d odom    = m_drivetrain.getState().Pose;
            double jump    = odom.getTranslation().getDistance(est.pose.getTranslation());
            double maxJump = m_isAutoMode ? kMaxPoseJumpAuto : kMaxPoseJumpTeleop;
            if (jump > maxJump) {
                putStatus("Jump " + String.format("%.2f", jump) + "m");
                return false;
            }
        }

        putStatus("OK " + est.tagCount + "t " + String.format("%.1f", est.avgTagDist) + "m");
        return true;
    }

    /**
     * Computes Kalman filter standard deviations scaled by tag distance and count.
     * Closer tags and more tags = lower std devs = more trust in the measurement.
     *
     * @return double[] { xyStdDev (meters), thetaStdDev (radians) }
     */
    private double[] computeStdDevs(LimelightHelpers.PoseEstimate est) {
        double distFactor = Math.pow(est.avgTagDist, kDistanceExp);
        double xy, theta;

        if (est.tagCount >= 2) {
            xy    = kMultiTagXyBase    * distFactor;
            theta = kMultiTagThetaBase * distFactor;
            if (est.tagCount >= 3) {
                xy    *= 0.5;
                theta *= 0.5;
            }
        } else {
            double ambig       = est.rawFiducials.length > 0 ? est.rawFiducials[0].ambiguity : kMaxAmbiguity;
            double ambigFactor = 1.0 + (ambig * 3.0);
            xy    = kSingleTagXyBase    * distFactor * ambigFactor;
            theta = kSingleTagThetaBase * distFactor * ambigFactor;
        }

        return new double[]{ Math.min(xy, 10.0), Math.min(theta, 10.0) };
    }

    /**
     * Validates a single pose estimate and feeds it to the Kalman filter if accepted.
     * Returns true if the measurement was accepted.
     */
    private boolean submitEstimate(LimelightHelpers.PoseEstimate est) {
        if (est == null || !isValidVisionMeasurement(est)) return false;
        double[] stdDevs = computeStdDevs(est);
        m_drivetrain.addVisionMeasurement(
            est.pose, est.timestampSeconds, stdDevs[0], stdDevs[1]);
        return true;
    }

    /**
     * Runs the full dual-camera pose fusion pipeline each loop.
     *
     * Both LL4 (rear) and LL2 (front) independently submit MegaTag2 estimates
     * to the drivetrain Kalman filter. Since they face opposite directions they
     * see different tags and provide complementary coverage:
     *   - Near the hub: LL2 sees Reef tags up close, LL4 may see nothing
     *   - Far from hub: LL4 sees field/alliance wall tags, LL2 may see nothing
     *   - Mid-field: both cameras may contribute simultaneously
     */
    private void processVisionMeasurements() {
        if (m_drivetrain == null || !m_visionUpdatesEnabled) {
            SmartDashboard.putBoolean("Vision/Active", false);
            return;
        }

        m_frameCount++;
        seedOrientation();

        // LL4 rear — cache estimate for telemetry reuse in periodic()
        m_latestMt2Rear = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kLimelight1);
        boolean rearAccepted = submitEstimate(m_latestMt2Rear);

        // LL2 front — independent estimate, not cached
        LimelightHelpers.PoseEstimate frontEst =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kLimelight2);
        boolean frontAccepted = submitEstimate(frontEst);

        // Update confidence timestamp whenever either camera contributes
        if (rearAccepted || frontAccepted) {
            m_lastVisionUpdateTime = Timer.getFPGATimestamp();
        }

        SmartDashboard.putBoolean("Vision/Active", rearAccepted || frontAccepted);

        // Detailed telemetry from whichever camera most recently had a valid estimate
        LimelightHelpers.PoseEstimate best = rearAccepted ? m_latestMt2Rear :
                                             frontAccepted ? frontEst : null;
        if (best != null) {
            SmartDashboard.putNumber("Vision/AvgTagDist_m", best.avgTagDist);
            SmartDashboard.putNumber("Vision/RotSpeed_dps", m_rotDegPerSec);
            if (best.tagCount == 1 && best.rawFiducials.length > 0) {
                SmartDashboard.putNumber("Vision/Ambiguity", best.rawFiducials[0].ambiguity);
                SmartDashboard.putNumber("Vision/TagID",     best.rawFiducials[0].id);
            }
        }

        SmartDashboard.putBoolean("Vision/LL4Active",  rearAccepted);
        SmartDashboard.putBoolean("Vision/LL2Active",  frontAccepted);
        SmartDashboard.putNumber("Vision/XY_StdDev",
            rearAccepted ? computeStdDevs(m_latestMt2Rear)[0] :
            frontAccepted ? computeStdDevs(frontEst)[0] : 0.0);
    }

    // =========================================================================
    // Pose reset — called from autonomousInit() and teleopInit()
    // =========================================================================

    /**
     * Attempts a hard pose reset using the best available MegaTag2 estimate.
     * Tries LL4 first, then LL2 if LL4 fails. Both cameras are seeded manually
     * since periodic() has not yet run.
     *
     * <p>Accepts single-tag resets given the limited tag visibility of the 2026 field.
     * Skips warmup, pose-jump, and rotation-speed checks since these are irrelevant
     * for a stationary hard reset.
     *
     * @return true if the pose was successfully reset, false otherwise
     */
    public boolean resetPoseFromVision() {
        if (m_drivetrain == null) {
            SmartDashboard.putString("AutoStart/PoseReset", "No drivetrain");
            return false;
        }

        double yaw = m_drivetrain.getState().Pose.getRotation().getDegrees();
        LimelightHelpers.SetRobotOrientation(kLimelight1,  yaw, 0, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation(kLimelight2, yaw, 0, 0, 0, 0, 0);

        // Try LL4 first, then fall back to LL2
        LimelightHelpers.PoseEstimate est =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kLimelight1);

        if (est == null || est.tagCount < kAutoResetMinTags || est.avgTagDist > kAutoResetMaxDist) {
            LimelightHelpers.PoseEstimate frontEst =
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kLimelight2);
            if (frontEst != null && frontEst.tagCount >= kAutoResetMinTags
                    && frontEst.avgTagDist <= kAutoResetMaxDist) {
                est = frontEst;
                SmartDashboard.putString("AutoStart/PoseReset", "Using LL2 for reset");
            }
        }

        if (est == null) {
            SmartDashboard.putString("AutoStart/PoseReset", "No estimate");
            SmartDashboard.putBoolean("AutoStart/PoseResetSucceeded", false);
            return false;
        }

        if (est.tagCount < kAutoResetMinTags) {
            SmartDashboard.putString("AutoStart/PoseReset",
                "Too few tags: " + est.tagCount + " (need " + kAutoResetMinTags + ")");
            SmartDashboard.putBoolean("AutoStart/PoseResetSucceeded", false);
            return false;
        }

        if (est.avgTagDist > kAutoResetMaxDist) {
            SmartDashboard.putString("AutoStart/PoseReset",
                "Tags too far: " + String.format("%.1f", est.avgTagDist) + "m");
            SmartDashboard.putBoolean("AutoStart/PoseResetSucceeded", false);
            return false;
        }

        if (est.rawFiducials.length > 0 && est.rawFiducials[0].ambiguity > kAutoResetMaxAmbig) {
            SmartDashboard.putString("AutoStart/PoseReset",
                "Ambiguity too high: " + String.format("%.2f", est.rawFiducials[0].ambiguity));
            SmartDashboard.putBoolean("AutoStart/PoseResetSucceeded", false);
            return false;
        }

        double px = est.pose.getX();
        double py = est.pose.getY();

        if (px < -kFieldMarginMeters || px > kFieldLengthMeters + kFieldMarginMeters
                || py < -kFieldMarginMeters || py > kFieldWidthMeters + kFieldMarginMeters
                || (px == 0.0 && py == 0.0)) {
            SmartDashboard.putString("AutoStart/PoseReset",
                "Invalid pose (" + String.format("%.2f, %.2f", px, py) + ")");
            SmartDashboard.putBoolean("AutoStart/PoseResetSucceeded", false);
            return false;
        }

        m_drivetrain.resetPose(est.pose);
        m_frameCount = 0;

        SmartDashboard.putString("AutoStart/PoseReset",
            "OK - " + est.tagCount + " tags @ " + String.format("%.2f", est.avgTagDist) + "m"
            + " -> (" + String.format("%.2f, %.2f", px, py) + ")");
        SmartDashboard.putBoolean("AutoStart/PoseResetSucceeded", true);
        SmartDashboard.putNumber("AutoStart/ResetPoseX", px);
        SmartDashboard.putNumber("AutoStart/ResetPoseY", py);

        return true;
    }

    // =========================================================================
    // Periodic
    // =========================================================================

    @Override
    public void periodic() {
        try {
            processVisionMeasurements();

            // Always-on telemetry — publishes every loop regardless of rejection filter
            SmartDashboard.putBoolean("Vision/HasTarget",       hasValidTarget());
            SmartDashboard.putBoolean("Vision/FrontHasTarget",  hasFrontTarget());
            SmartDashboard.putBoolean("Vision/PoseConfident",   isPoseConfident());
            SmartDashboard.putNumber("Vision/HorizontalOffset", getHorizontalOffset());
            SmartDashboard.putNumber("Vision/Distance",         getDistanceToTarget());

            // true when LL2 is providing distance, false when falling back to LL4
            SmartDashboard.putBoolean("Vision/UsingFrontLL",
                LimelightHelpers.getRawFiducials(kLimelight2).length > 0);

            // Tag count and pose from cached LL4 estimate
            if (m_latestMt2Rear != null) {
                SmartDashboard.putNumber("Vision/TagCount", m_latestMt2Rear.tagCount);
                SmartDashboard.putNumber("Vision/PoseX",    m_latestMt2Rear.pose.getX());
                SmartDashboard.putNumber("Vision/PoseY",    m_latestMt2Rear.pose.getY());
                if (m_latestMt2Rear.tagCount > 0) {
                    m_visionField.setRobotPose(m_latestMt2Rear.pose);
                }
            } else {
                SmartDashboard.putNumber("Vision/TagCount", 0);
                SmartDashboard.putNumber("Vision/PoseX",    0.0);
                SmartDashboard.putNumber("Vision/PoseY",    0.0);
            }
        } catch (Exception e) {
            // Prevent any limelight NetworkTables error from crashing the robot
        }
    }
}