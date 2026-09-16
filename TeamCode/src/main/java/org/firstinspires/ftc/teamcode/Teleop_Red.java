package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;

import java.util.List;

/**
 * TeleOp Control Scheme:
 * 
 * DRIVETRAIN:
 * - Left stick: translate (field-centric)
 * - Right stick X: rotate
 * 
 * INTAKE SYSTEM:
 * - On start: intakeIdle
 * - Right bumper: toggle between intakeOn and intakeIdle
 * - Left bumper (hold): intakeReverse, release returns to previous state
 * 
 * SCORING SYSTEM:
 * - Left trigger (hold): auto-align to closer hive using follower.hold(), spin up flywheel
 *   - Still allows translation with heading lock
 *   - Rumbles both gamepads when aligned (200ms)
 * - Right trigger (press): check alignment & flywheel velocity, then transferOn
 *   - Release: flywheelOff
 * 
 * TELEMETRY:
 * - ALIGNED status
 * - Robot pose (x, y, heading)
 * - Flywheel velocity
 * - Distance to target hive
 * - Angle difference to target
 * - Target hive pose
 */
@Configurable
@TeleOp(name = "TeleOp_Red", group = "Teleop")
public class Teleop_Red extends OpMode {

    // Tunable heading control for manual heading lock
    public static double HEADING_KP = 2.0;
    public static double HEADING_KI = 0.0;
    public static double HEADING_KD = 0.1;
    
    private RobotHardware robot;
    private RunToPose runToPose;
    private List<LynxModule> allHubs;
    
    // Intake state management
    private enum IntakeState { IDLE, ON }
    private IntakeState intakeState = IntakeState.IDLE;
    private IntakeState savedIntakeState = IntakeState.IDLE;
    
    // Button edge detection
    private boolean previousRightBumper = false;
    private boolean previousLeftBumper = false;
    private boolean previousLeftTrigger = false;
    private boolean previousRightTrigger = false;
    private boolean lastTouchpad = false;
    private boolean lastShare = false;
    
    // Alignment state
    private boolean isAligning = false;
    private Pose targetHive = null;
    private boolean hasRumbled = false;
    
    // Heading lock controller for manual drive during alignment
    private com.pedropathing.controllers.PIDController headingController;
    
    @Override
    public void init() {
        // Initialize robot hardware
        robot = new RobotHardware();
        robot.init(hardwareMap);
        
        // Initialize RunToPose
        runToPose = new RunToPose(robot.follower, this);
        
        // Initialize heading controller for manual drive during alignment
        headingController = new com.pedropathing.controllers.PIDController(HEADING_KP, HEADING_KI, HEADING_KD);
        
        // Set up bulk read caching for performance
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        robot.follower.setPose(robot.RED_START);

        robot.follower.manual();
        
        PanelsTelemetry.INSTANCE.getTelemetry().debug("Status: Initialized");
        PanelsTelemetry.INSTANCE.getTelemetry().update();
    }
    
    @Override
    public void start() {
        // Start with intake idle
        robot.intakeIdle();
        intakeState = IntakeState.IDLE;
    }
    
    @Override
    public void loop() {
        // Clear bulk cache at start of each loop for fresh sensor reads
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
        
        // Update Pedro Pathing follower (CRITICAL)
        robot.follower.update();
        
        Pose currentPose = robot.follower.pose();
        
        // ===== INTAKE CONTROL =====
        handleIntakeControl();

        handleGamepad();
        
        // ===== ALIGNMENT & SCORING CONTROL =====
        handleScoringControl(currentPose);
        
        // ===== DRIVETRAIN CONTROL =====
        handleDriveControl(currentPose);
        
        // ===== TELEMETRY =====
        updateTelemetry(currentPose);
    }

    private void handleGamepad() {
        // Read button states
        boolean currentTouchpad = gamepad1.touchpad;
        boolean currentShare = gamepad1.share; // also gamepad1.back on standard Xbox/Logitech layout

        // --- TOUCHPAD: Reset Pose (0, 0, 0) AND IMU ---
        if (currentTouchpad && !lastTouchpad) {
            // Reset the follower pose directly
            robot.follower.setPose(new Pose(72, 72, 90));
        }

        // --- SHARE: Reset IMU ONLY (Keep current X and Y position) ---
        if (currentShare && !lastShare) {
            // Preserve existing X and Y, zero out heading
            Pose currentPose = robot.follower.pose();
            robot.follower.setPose(new Pose(currentPose.x(), currentPose.y(), 90));
        }

        // Update edge detection states
        lastTouchpad = currentTouchpad;
        lastShare = currentShare;
    }
    private void handleIntakeControl() {
        // Right bumper: toggle intake on/off
        boolean currentRightBumper = gamepad1.right_bumper;
        if (currentRightBumper && !previousRightBumper) {
            if (intakeState == IntakeState.IDLE) {
                robot.intakeOn();
                intakeState = IntakeState.ON;
            } else {
                robot.intakeIdle();
                intakeState = IntakeState.IDLE;
            }
            savedIntakeState = intakeState;
        }
        previousRightBumper = currentRightBumper;
        
        // Left bumper: hold to reverse, release to return to previous state
        boolean currentLeftBumper = gamepad1.left_bumper;
        if (currentLeftBumper && !previousLeftBumper) {
            // Just pressed - start reversing
            savedIntakeState = intakeState;
            robot.intakeReverse();
        } else if (!currentLeftBumper && previousLeftBumper) {
            // Just released - return to saved state
            if (savedIntakeState == IntakeState.ON) {
                robot.intakeOn();
                intakeState = IntakeState.ON;
            } else {
                robot.intakeIdle();
                intakeState = IntakeState.IDLE;
            }
        }
        previousLeftBumper = currentLeftBumper;
    }
    
    private void handleScoringControl(Pose currentPose) {
        boolean currentLeftTrigger = gamepad1.left_trigger > 0.1;
        boolean currentRightTrigger = gamepad1.right_trigger > 0.1;
        
        // Left trigger: align to hive using follower.hold() and spin up flywheel (edge detection)
        if (currentLeftTrigger && !previousLeftTrigger) {
            // Just pressed - start aligning and find closer hive
            double distLeft = currentPose.toVector2D().distance(RobotHardware.RED_HIVE_LEFT.toVector2D());
            double distRight = currentPose.toVector2D().distance(RobotHardware.RED_HIVE_RIGHT.toVector2D());
            targetHive = distLeft < distRight ? RobotHardware.RED_HIVE_LEFT : RobotHardware.RED_HIVE_RIGHT;
            
            isAligning = true;
            hasRumbled = false;
            headingController.reset();
        }
        
        // While left trigger held: maintain alignment
        if (currentLeftTrigger) {
            if (targetHive != null) {
                // Calculate target heading to face hive
                Vector2D toHive = targetHive.toVector2D().minus(currentPose.toVector2D());
                double targetHeading = toHive.theta();
                
                // Check if aligned
                double headingError = com.pedropathing.utils.Angle.normalizeSigned(targetHeading - currentPose.heading());
                boolean aligned = Math.abs(Math.toDegrees(headingError)) < RobotHardware.ALIGNMENT_TOLERANCE;
                
                // Rumble when first aligned
                if (aligned && !hasRumbled) {
                    gamepad1.rumble(200);
                    gamepad2.rumble(200);
                    hasRumbled = true;
                    robot.flywheelOn();
                }
                
                // FIX #1: Keep flywheel spinning ONLY while aligned (stop if alignment lost)
                if (aligned) {
                    robot.flywheelOn();
                } else {
                    // Lost alignment - stop flywheel
                    robot.flywheelOff();
                }
            }
        } else if (!currentLeftTrigger && previousLeftTrigger) {
            // Just released left trigger - stop aligning, clear state, and reset heading controller
            isAligning = false;
            targetHive = null;
            hasRumbled = false;
            robot.flywheelOff();
            headingController.reset();  // Reset on release to clear accumulated error
        }
        
        previousLeftTrigger = currentLeftTrigger;
        
        // Right trigger: fire if aligned and flywheel ready
        if (currentRightTrigger && !previousRightTrigger) {
            // Just pressed - check conditions
            if (isAligning && targetHive != null) {
                // Calculate alignment
                Vector2D toHive = targetHive.toVector2D().minus(currentPose.toVector2D());
                double targetHeading = toHive.theta();
                double headingError = com.pedropathing.utils.Angle.normalizeSigned(targetHeading - currentPose.heading());
                
                boolean aligned = Math.abs(Math.toDegrees(headingError)) < RobotHardware.ALIGNMENT_TOLERANCE;
                boolean flywheelReady = robot.isFlywheelReady();
                
                if (aligned && flywheelReady) {
                    robot.transferOn();
                }
            }
        } else if (!currentRightTrigger && previousRightTrigger) {
            // Just released - stop flywheel
            robot.flywheelOff();
        }
        previousRightTrigger = currentRightTrigger;
    }
    
    private void handleDriveControl(Pose currentPose) {
        // Get driver inputs
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        
        // Create base drive powers
        DrivePowers powers = new DrivePowers(forward, strafe, rotate);
        
        // If aligning, apply heading lock using ManualDrive.headingLock()
        if (isAligning && targetHive != null) {
            // Calculate target heading to hive
            Vector2D toHive = targetHive.toVector2D().minus(currentPose.toVector2D());
            double targetHeading = toHive.theta();
            
            // Use Pedro Pathing's headingLock with PID controller
            powers = ManualDrive.headingLock(robot.follower, headingController, powers, targetHeading);
        }
        
        // Apply field-centric drive using ManualDrive
        powers = ManualDrive.fieldCentric(powers, currentPose.heading());
        
        // Send powers to follower using manual mode
        robot.follower.manual(powers);
    }
    
    private void updateTelemetry(Pose currentPose) {
        // Alignment status
        if (isAligning && targetHive != null) {
            // Calculate alignment using Vector2D
            Vector2D toHive = targetHive.toVector2D().minus(currentPose.toVector2D());
            double targetHeading = toHive.theta();
            double headingError = com.pedropathing.utils.Angle.normalizeSigned(targetHeading - currentPose.heading());
            
            boolean aligned = Math.abs(Math.toDegrees(headingError)) < RobotHardware.ALIGNMENT_TOLERANCE;
            if (aligned) {
                PanelsTelemetry.INSTANCE.getTelemetry().debug("===== ALIGNED =====");
            } else {
                PanelsTelemetry.INSTANCE.getTelemetry().debug("Aligning...");
            }
        }
        
        // Robot pose
        PanelsTelemetry.INSTANCE.getTelemetry().debug(String.format(
            "Pose: X: %.1f, Y: %.1f, H: %.1f°",
            currentPose.x(),
            currentPose.y(),
            Math.toDegrees(currentPose.heading())
        ));
        
        // Flywheel velocity
        PanelsTelemetry.INSTANCE.getTelemetry().debug(String.format(
            "Flywheel Velocity: %.0f ticks/sec", 
            robot.getFlywheelVelocity()
        ));
        PanelsTelemetry.INSTANCE.getTelemetry().debug(
            "Flywheel Ready: " + robot.isFlywheelReady()
        );
        
        // Target hive info
        if (targetHive != null) {
            Vector2D toHive = targetHive.toVector2D().minus(currentPose.toVector2D());
            double distance = toHive.magnitude();
            double targetHeading = toHive.theta();
            double headingError = com.pedropathing.utils.Angle.normalizeSigned(targetHeading - currentPose.heading());
            
            PanelsTelemetry.INSTANCE.getTelemetry().debug(String.format(
                "Target Hive: X: %.1f, Y: %.1f",
                targetHive.x(),
                targetHive.y()
            ));
            PanelsTelemetry.INSTANCE.getTelemetry().debug(String.format(
                "Distance to Target: %.1f",
                distance
            ));
            PanelsTelemetry.INSTANCE.getTelemetry().debug(String.format(
                "Angle Difference: %.1f°",
                Math.toDegrees(headingError)
            ));
        } else {
            PanelsTelemetry.INSTANCE.getTelemetry().debug("Target Hive: None");
        }
        
        // Intake state
        PanelsTelemetry.INSTANCE.getTelemetry().debug("Intake State: " + intakeState);
        PanelsTelemetry.INSTANCE.getTelemetry().debug("Is Aligning: " + isAligning);
        PanelsTelemetry.INSTANCE.getTelemetry().debug("Follower State: " + robot.follower.mode());
        
        PanelsTelemetry.INSTANCE.getTelemetry().update();
    }
    
    @Override
    public void stop() {
        robot.intakeIdle();
        robot.flywheelOff();
    }
}
