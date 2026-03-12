package frc.robot.subsystems.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.limelight.LimelightHelpers.RawFiducial;

/** Manages Limelight vision, MegaTag2 odometry fusion, distance measurement, and AprilTag alignment. */
public class LimelightSubsystem extends SubsystemBase {

    private static final String kLimelightName = "limelight";

    // Proportional gain for rotation-to-target alignment
    private static final double kPAim = 0.035;

    // Rejection thresholds
    private static final double kMaxAmbiguity         = 0.2;   // single-tag ambiguity ratio (0=perfect, 1=ambiguous)
    private static final double kMaxSingleTagDistance = 4.0;   // meters
    private static final double kMaxMultiTagDistance  = 6.0;   // meters
    private static final double kMaxPoseJumpTeleop    = 0.75;  // meters
    private static final double kMaxPoseJumpAuto      = 0.5;   // meters
    private static final double kMaxRotationDegPerSec = 720.0; // deg/s

    // 2026 field dimensions (meters)
    private static final double kFieldLengthMeters = 17.548;
    private static final double kFieldWidthMeters  = 8.211;
    private static final double kFieldMarginMeters = 0.5;

    // Standard deviation base values for the pose estimator (higher = less trust in vision)
    private static final double kSingleTagXyBase    = 0.5;
    private static final double kSingleTagThetaBase = 0.9;
    private static final double kMultiTagXyBase     = 0.2;
    private static final double kMultiTagThetaBase  = 0.4;
    private static final double kDistanceExp        = 2.0; // stddev scales as distance^kDistanceExp

    // Frames to skip after a mode change before applying vision corrections (~1 sec at 50 Hz)
    private static final int kWarmupFrames = 50;

    // Minimum quality thresholds for an auto start pose reset —
    // stricter than normal fusion since we are committing to this as the starting pose
    private static final int    kAutoResetMinTags  = 2;
    private static final double kAutoResetMaxDist  = 3.5;  // meters
    private static final double kAutoResetMaxAmbig = 0.15; // tighter than kMaxAmbiguity

    private CommandSwerveDrivetrain m_drivetrain;
    private boolean m_visionUpdatesEnabled = true;
    private boolean m_isAutoMode           = false;

    private double m_lastYawDegrees = 0.0;
    private double m_lastTimestamp  = 0.0;
    private double m_rotDegPerSec   = 0.0;
    private int    m_frameCount     = 0;

    // Cached each loop to avoid double NetworkTables reads
    private LimelightHelpers.PoseEstimate m_latestMt2 = null;

    // Field widget for Elastic — shows raw vision pose alongside odometry pose
    private final Field2d m_visionField = new Field2d();

    public LimelightSubsystem() {
        LimelightHelpers.setLEDMode_ForceOff(kLimelightName);
        SmartDashboard.putData("Vision/Field", m_visionField);
    }

    /** Sets the drivetrain reference required for MegaTag2 odometry fusion. */
    public void setDrivetrain(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
    }

    /** Enables or disables vision odometry updates. */
    public void setVisionUpdatesEnabled(boolean enabled) {
        m_visionUpdatesEnabled = enabled;
    }

    /**
     * Switches between auto and teleop vision thresholds. Resets the warmup counter.
     *
     * @param isAuto true for autonomous mode, false for teleop
     */
    public void setAutoMode(boolean isAuto) {
        m_isAutoMode = isAuto;
        m_frameCount = 0;
        if (isAuto) {
            setLEDsOn();
        } else {
            setLEDsOff();
        }
    }

    /** Returns horizontal offset from crosshair to target in degrees. Positive = right. */
    public double getHorizontalOffset() {
        return LimelightHelpers.getTX(kLimelightName);
    }

    /** Returns vertical offset from crosshair to target in degrees. */
    public double getVerticalOffset() {
        return LimelightHelpers.getTY(kLimelightName);
    }

    /** Returns target area as a percentage of the image (0-100%). */
    public double getTargetArea() {
        return LimelightHelpers.getTA(kLimelightName);
    }

    /** Returns true if the Limelight has a valid target lock. */
    public boolean hasValidTarget() {
        return LimelightHelpers.getTV(kLimelightName);
    }

    /**
     * Returns distance from the camera lens to the primary AprilTag in meters.
     * Uses distToCamera from the 3D fiducial solve — matches tape measure calibration
     * data measured from the camera lens, not the robot center.
     *
     * @return distance in meters, or -1.0 if no tag is visible
     */
    public double getDistanceToTarget() {
        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(kLimelightName);
        if (fiducials.length == 0) return -1.0;
        return fiducials[0].distToCamera;
    }

    /**
     * Returns distance to the primary AprilTag, falling back to 3.0 m if the
     * measurement is missing or out of range.
     */
    public double getDistanceToTargetSafe() {
        double d = getDistanceToTarget();
        if (d < 0 || d > 10.0 || Double.isNaN(d) || Double.isInfinite(d)) return 3.0;
        return d;
    }

    /**
     * Returns a rotational rate for aligning to the current target.
     *
     * @param maxAngularRate the robot's maximum angular rate in radians per second
     * @return rotational rate in radians per second, or 0 if no target is visible
     */
    public double calculateAimVelocity(double maxAngularRate) {
        if (!hasValidTarget()) return 0.0;
        return -getHorizontalOffset() * kPAim * maxAngularRate;
    }

    /** Returns the latest cached MegaTag2 pose estimate. Updated once per loop in periodic(). */
    public LimelightHelpers.PoseEstimate getMegaTag2Estimate() {
        return m_latestMt2;
    }

    /**
     * Sends the robot's current gyro yaw to Limelight and computes rotation speed.
     * Must be called before reading the MegaTag2 estimate each loop.
     */
    private void seedOrientation() {
        if (m_drivetrain == null) return;

        double yaw = m_drivetrain.getState().Pose.getRotation().getDegrees();
        double now = Timer.getFPGATimestamp();

        if (m_lastTimestamp > 0.0) {
            double dt = now - m_lastTimestamp;
            if (dt > 0.0) {
                double delta = yaw - m_lastYawDegrees;
                // Wrap to [-180, 180]
                while (delta >  180.0) delta -= 360.0;
                while (delta < -180.0) delta += 360.0;
                m_rotDegPerSec = Math.abs(delta / dt);
            }
        }

        m_lastYawDegrees = yaw;
        m_lastTimestamp  = now;

        // Flush ensures the yaw reaches Limelight before the estimate is read this loop
        LimelightHelpers.SetRobotOrientation(kLimelightName, yaw, 0, 0, 0, 0, 0);
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

        // MegaTag2 yaw seed becomes stale during fast rotation
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
     * Computes pose estimator standard deviations scaled by distance and tag count.
     *
     * @return double[] containing {xyStdDev (meters), thetaStdDev (radians)}
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
     * Attempts to reset the drivetrain pose using a fresh MegaTag2 estimate.
     *
     * <p>Intended to be called once in autonomousInit() and teleopInit() before any
     * commands are scheduled. Seeds the Limelight orientation manually since periodic()
     * has not yet run.
     *
     * <p>Requires at least {@value kAutoResetMinTags} tags within {@value kAutoResetMaxDist} m.
     * Skips the warmup, pose-jump, and rotation-speed checks since those are irrelevant
     * for a stationary hard pose reset.
     *
     * @return true if a valid estimate was found and the pose was reset, false otherwise
     */
    public boolean resetPoseFromVision() {
        if (m_drivetrain == null) {
            SmartDashboard.putString("AutoStart/PoseReset", "No drivetrain");
            return false;
        }

        double yaw = m_drivetrain.getState().Pose.getRotation().getDegrees();
        LimelightHelpers.SetRobotOrientation(kLimelightName, yaw, 0, 0, 0, 0, 0);

        LimelightHelpers.PoseEstimate est =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kLimelightName);

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

        if (est.tagCount == 1 && est.rawFiducials.length > 0
                && est.rawFiducials[0].ambiguity > kAutoResetMaxAmbig) {
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

        // Reset warmup so normal Kalman corrections start fresh
        m_frameCount = 0;

        SmartDashboard.putString("AutoStart/PoseReset",
            "OK - " + est.tagCount + " tags @ " + String.format("%.2f", est.avgTagDist) + "m"
            + " -> (" + String.format("%.2f, %.2f", px, py) + ")");
        SmartDashboard.putBoolean("AutoStart/PoseResetSucceeded", true);
        SmartDashboard.putNumber("AutoStart/ResetPoseX", px);
        SmartDashboard.putNumber("AutoStart/ResetPoseY", py);

        return true;
    }

    private void processVisionMeasurements() {
        if (m_drivetrain == null || !m_visionUpdatesEnabled) {
            SmartDashboard.putBoolean("Vision/Active", false);
            return;
        }

        m_frameCount++;
        seedOrientation();

        // Fetch once and cache — reused in periodic() for always-on telemetry
        m_latestMt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kLimelightName);

        if (m_latestMt2 == null || !isValidVisionMeasurement(m_latestMt2)) {
            SmartDashboard.putBoolean("Vision/Active", false);
            return;
        }

        double[] stdDevs = computeStdDevs(m_latestMt2);
        m_drivetrain.addVisionMeasurement(
            m_latestMt2.pose,
            m_latestMt2.timestampSeconds,
            stdDevs[0],
            stdDevs[1]
        );

        SmartDashboard.putBoolean("Vision/Active",      true);
        SmartDashboard.putNumber("Vision/AvgTagDist_m", m_latestMt2.avgTagDist);
        SmartDashboard.putNumber("Vision/XY_StdDev",    stdDevs[0]);
        SmartDashboard.putNumber("Vision/Theta_StdDev", stdDevs[1]);
        SmartDashboard.putNumber("Vision/RotSpeed_dps", m_rotDegPerSec);

        if (m_latestMt2.tagCount == 1 && m_latestMt2.rawFiducials.length > 0) {
            SmartDashboard.putNumber("Vision/Ambiguity", m_latestMt2.rawFiducials[0].ambiguity);
            SmartDashboard.putNumber("Vision/TagID",     m_latestMt2.rawFiducials[0].id);
        }
    }

    private void putStatus(String msg) {
        SmartDashboard.putString("Vision/Status", msg);
    }

    public void setLEDsOn()  { LimelightHelpers.setLEDMode_ForceOn(kLimelightName);  }
    public void setLEDsOff() { LimelightHelpers.setLEDMode_ForceOff(kLimelightName); }

    public void setPipeline(int pipeline) {
        LimelightHelpers.setPipelineIndex(kLimelightName, pipeline);
    }

    @Override
    public void periodic() {
        try {
            processVisionMeasurements();

            // Always-on telemetry — publishes every loop regardless of rejection filter
            SmartDashboard.putBoolean("Vision/HasTarget",       hasValidTarget());
            SmartDashboard.putNumber("Vision/HorizontalOffset", getHorizontalOffset());
            SmartDashboard.putNumber("Vision/Distance",         getDistanceToTarget());

            // Use cached m_latestMt2 — avoids a second NetworkTables read this loop
            if (m_latestMt2 != null) {
                SmartDashboard.putNumber("Vision/TagCount", m_latestMt2.tagCount);
                SmartDashboard.putNumber("Vision/PoseX",    m_latestMt2.pose.getX());
                SmartDashboard.putNumber("Vision/PoseY",    m_latestMt2.pose.getY());
                if (m_latestMt2.tagCount > 0) {
                    m_visionField.setRobotPose(m_latestMt2.pose);
                }
            } else {
                SmartDashboard.putNumber("Vision/TagCount", 0);
                SmartDashboard.putNumber("Vision/PoseX",    0.0);
                SmartDashboard.putNumber("Vision/PoseY",    0.0);
            }
        } catch (Exception e) {
            // Prevent a Limelight error from crashing the robot
        }
    }
}