package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import org.junit.Test;
import org.junit.Before;
import static org.junit.Assert.*;

/**
 * Unit tests for teleop control logic including:
 * - Alignment calculations and checks
 * - Target selection logic
 * - Scoring conditions
 * - Intake state transitions
 * - Gamepad input mocking and edge detection
 * - Flywheel state transitions
 */
public class TeleopControlTest {
    
    private MockGamepadState gamepad;
    private MockRobotState robot;
    
    @Before
    public void setUp() {
        gamepad = new MockGamepadState();
        robot = new MockRobotState();
    }
    
    // ===== TARGET HIVE SELECTION TESTS =====
    
    @Test
    public void testSelectCloserHive_LeftIsCloser() {
        // Robot closer to left hive
        Pose robotPose = new Pose(10, 50, 0);
        Pose leftHive = new Pose(5, 50, 0);   // Distance: 5
        Pose rightHive = new Pose(100, 50, 0); // Distance: 90
        
        Pose selected = selectCloserHive(robotPose, leftHive, rightHive);
        assertEquals(leftHive, selected);
    }
    
    @Test
    public void testSelectCloserHive_RightIsCloser() {
        // Robot closer to right hive
        Pose robotPose = new Pose(95, 50, 0);
        Pose leftHive = new Pose(5, 50, 0);   // Distance: 90
        Pose rightHive = new Pose(100, 50, 0); // Distance: 5
        
        Pose selected = selectCloserHive(robotPose, leftHive, rightHive);
        assertEquals(rightHive, selected);
    }
    
    @Test
    public void testSelectCloserHive_Equidistant() {
        // Robot equidistant from both hives
        Pose robotPose = new Pose(50, 50, 0);
        Pose leftHive = new Pose(30, 50, 0);   // Distance: 20
        Pose rightHive = new Pose(70, 50, 0);  // Distance: 20
        
        Pose selected = selectCloserHive(robotPose, leftHive, rightHive);
        // When equidistant, the implementation returns left because distLeft < distRight is false
        // but distLeft is not > distRight either, so we need to check which one is actually returned
        // The condition is distLeft < distRight, which is false, so rightHive is returned
        assertEquals(rightHive, selected);
    }
    
    @Test
    public void testSelectCloserHive_DiagonalPositions() {
        // Robot at diagonal position
        Pose robotPose = new Pose(40, 40, 0);
        Pose leftHive = new Pose(10, 60, Math.PI / 4);
        Pose rightHive = RobotHardware.RED_HIVE_RIGHT;
        
        double distLeft = distance(robotPose, leftHive);
        double distRight = distance(robotPose, rightHive);
        
        Pose selected = selectCloserHive(robotPose, leftHive, rightHive);
        Pose expected = distLeft < distRight ? leftHive : rightHive;
        assertEquals(expected, selected);
    }
    
    // ===== ALIGNMENT CALCULATION TESTS =====
    
    @Test
    public void testCalculateTargetHeading_DirectlyEast() {
        // Target directly east of robot
        Pose robotPose = new Pose(10, 50, 0);
        Pose targetPose = new Pose(20, 50, 0);
        
        double targetHeading = calculateTargetHeading(robotPose, targetPose);
        assertEquals(0.0, targetHeading, 0.01);
    }
    
    @Test
    public void testCalculateTargetHeading_DirectlyNorth() {
        // Target directly north of robot
        Pose robotPose = new Pose(50, 10, 0);
        Pose targetPose = new Pose(50, 20, 0);
        
        double targetHeading = calculateTargetHeading(robotPose, targetPose);
        assertEquals(Math.PI / 2, targetHeading, 0.01);
    }
    
    @Test
    public void testCalculateTargetHeading_DirectlyWest() {
        // Target directly west of robot
        Pose robotPose = new Pose(20, 50, 0);
        Pose targetPose = new Pose(10, 50, 0);
        
        double targetHeading = calculateTargetHeading(robotPose, targetPose);
        assertEquals(Math.PI, Math.abs(targetHeading), 0.01);
    }
    
    @Test
    public void testCalculateTargetHeading_DirectlySouth() {
        // Target directly south of robot
        Pose robotPose = new Pose(50, 20, 0);
        Pose targetPose = new Pose(50, 10, 0);
        
        double targetHeading = calculateTargetHeading(robotPose, targetPose);
        assertEquals(-Math.PI / 2, targetHeading, 0.01);
    }
    
    @Test
    public void testCalculateTargetHeading_Diagonal45() {
        // Target at 45 degrees northeast
        Pose robotPose = new Pose(0, 0, 0);
        Pose targetPose = new Pose(10, 10, 0);
        
        double targetHeading = calculateTargetHeading(robotPose, targetPose);
        assertEquals(Math.PI / 4, targetHeading, 0.01);
    }
    
    // ===== ALIGNMENT CHECK TESTS =====
    
    @Test
    public void testIsAligned_PerfectAlignment() {
        // Robot perfectly aligned with target
        double currentHeading = Math.PI / 4;
        double targetHeading = Math.PI / 4;
        
        boolean aligned = isAligned(currentHeading, targetHeading, RobotHardware.ALIGNMENT_TOLERANCE);
        assertTrue("Perfect alignment should be aligned", aligned);
    }
    
    @Test
    public void testIsAligned_WithinTolerance() {
        // Robot within tolerance
        double currentHeading = Math.toRadians(45);
        double targetHeading = Math.toRadians(46);  // 1 degree off
        
        boolean aligned = isAligned(currentHeading, targetHeading, RobotHardware.ALIGNMENT_TOLERANCE);
        assertTrue("Within tolerance should be aligned", aligned);
    }
    
    @Test
    public void testIsAligned_JustOutsideTolerance() {
        // Robot just outside tolerance
        double currentHeading = Math.toRadians(45);
        double targetHeading = Math.toRadians(48);  // 3 degrees off (tolerance is 2)
        
        boolean aligned = isAligned(currentHeading, targetHeading, RobotHardware.ALIGNMENT_TOLERANCE);
        assertFalse("Outside tolerance should not be aligned", aligned);
    }
    
    @Test
    public void testIsAligned_ExactlyAtTolerance() {
        // Robot exactly at tolerance boundary (exclusive comparison, so just outside)
        double currentHeading = Math.toRadians(45);
        double targetHeading = Math.toRadians(45 + RobotHardware.ALIGNMENT_TOLERANCE);
        
        boolean aligned = isAligned(currentHeading, targetHeading, RobotHardware.ALIGNMENT_TOLERANCE);
        assertFalse("At tolerance boundary should not be aligned (exclusive)", aligned);
    }
    
    @Test
    public void testIsAligned_WrapAroundPositive() {
        // Test wrap-around from 179° to -179°
        double currentHeading = Math.toRadians(179);
        double targetHeading = Math.toRadians(-179);  // Actually only 2° apart
        
        boolean aligned = isAligned(currentHeading, targetHeading, RobotHardware.ALIGNMENT_TOLERANCE);
        assertTrue("Wrap-around alignment should work", aligned);
    }
    
    @Test
    public void testIsAligned_WrapAroundNegative() {
        // Test wrap-around from -179° to 179°
        double currentHeading = Math.toRadians(-179);
        double targetHeading = Math.toRadians(179);  // Actually only 2° apart
        
        boolean aligned = isAligned(currentHeading, targetHeading, RobotHardware.ALIGNMENT_TOLERANCE);
        assertTrue("Reverse wrap-around alignment should work", aligned);
    }
    
    @Test
    public void testIsAligned_Opposite() {
        // Robot facing opposite direction
        double currentHeading = 0;
        double targetHeading = Math.PI;
        
        boolean aligned = isAligned(currentHeading, targetHeading, RobotHardware.ALIGNMENT_TOLERANCE);
        assertFalse("Opposite direction should not be aligned", aligned);
    }
    
    // ===== SCORING CONDITIONS TESTS =====
    
    @Test
    public void testCanFire_AllConditionsMet() {
        // Aligned, flywheel ready, aligning mode active
        boolean isAligning = true;
        boolean aligned = true;
        boolean flywheelReady = true;
        
        boolean canFire = canFire(isAligning, aligned, flywheelReady);
        assertTrue("Should be able to fire when all conditions met", canFire);
    }
    
    @Test
    public void testCanFire_NotAligning() {
        // Not in aligning mode
        boolean isAligning = false;
        boolean aligned = true;
        boolean flywheelReady = true;
        
        boolean canFire = canFire(isAligning, aligned, flywheelReady);
        assertFalse("Cannot fire when not aligning", canFire);
    }
    
    @Test
    public void testCanFire_NotAligned() {
        // Not aligned
        boolean isAligning = true;
        boolean aligned = false;
        boolean flywheelReady = true;
        
        boolean canFire = canFire(isAligning, aligned, flywheelReady);
        assertFalse("Cannot fire when not aligned", canFire);
    }
    
    @Test
    public void testCanFire_FlywheelNotReady() {
        // Flywheel not ready
        boolean isAligning = true;
        boolean aligned = true;
        boolean flywheelReady = false;
        
        boolean canFire = canFire(isAligning, aligned, flywheelReady);
        assertFalse("Cannot fire when flywheel not ready", canFire);
    }
    
    @Test
    public void testCanFire_NoConditionsMet() {
        // No conditions met
        boolean isAligning = false;
        boolean aligned = false;
        boolean flywheelReady = false;
        
        boolean canFire = canFire(isAligning, aligned, flywheelReady);
        assertFalse("Cannot fire when no conditions met", canFire);
    }
    
    // ===== DISTANCE CALCULATION TESTS =====
    
    @Test
    public void testDistance_SamePoint() {
        Pose p1 = new Pose(10, 20, 0);
        Pose p2 = new Pose(10, 20, 0);
        
        double dist = distance(p1, p2);
        assertEquals(0.0, dist, 0.001);
    }
    
    @Test
    public void testDistance_HorizontalLine() {
        Pose p1 = new Pose(0, 10, 0);
        Pose p2 = new Pose(10, 10, 0);
        
        double dist = distance(p1, p2);
        assertEquals(10.0, dist, 0.001);
    }
    
    @Test
    public void testDistance_VerticalLine() {
        Pose p1 = new Pose(10, 0, 0);
        Pose p2 = new Pose(10, 10, 0);
        
        double dist = distance(p1, p2);
        assertEquals(10.0, dist, 0.001);
    }
    
    @Test
    public void testDistance_Diagonal() {
        Pose p1 = new Pose(0, 0, 0);
        Pose p2 = new Pose(3, 4, 0);
        
        double dist = distance(p1, p2);
        assertEquals(5.0, dist, 0.001);  // 3-4-5 triangle
    }
    
    // ===== INTAKE STATE TRANSITION TESTS =====
    
    @Test
    public void testIntakeToggle_IdleToOn() {
        // Simulating toggle from IDLE to ON
        String currentState = "IDLE";
        String newState = toggleIntake(currentState);
        assertEquals("ON", newState);
    }
    
    @Test
    public void testIntakeToggle_OnToIdle() {
        // Simulating toggle from ON to IDLE
        String currentState = "ON";
        String newState = toggleIntake(currentState);
        assertEquals("IDLE", newState);
    }
    
    @Test
    public void testIntakeReverse_SavesState() {
        // When reversing, original state should be saved
        String originalState = "ON";
        String savedState = saveIntakeState(originalState);
        assertEquals(originalState, savedState);
    }
    
    // ===== HEADING ERROR NORMALIZATION TESTS =====
    
    @Test
    public void testNormalizeHeadingError_SmallPositive() {
        double error = Math.toRadians(30);
        double normalized = normalizeAngleSigned(error);
        assertEquals(Math.toRadians(30), normalized, 0.001);
    }
    
    @Test
    public void testNormalizeHeadingError_SmallNegative() {
        double error = Math.toRadians(-30);
        double normalized = normalizeAngleSigned(error);
        assertEquals(Math.toRadians(-30), normalized, 0.001);
    }
    
    @Test
    public void testNormalizeHeadingError_LargePositive() {
        // 270° should become -90°
        double error = Math.toRadians(270);
        double normalized = normalizeAngleSigned(error);
        assertEquals(Math.toRadians(-90), normalized, 0.001);
    }
    
    @Test
    public void testNormalizeHeadingError_LargeNegative() {
        // -270° should become 90°
        double error = Math.toRadians(-270);
        double normalized = normalizeAngleSigned(error);
        assertEquals(Math.toRadians(90), normalized, 0.001);
    }
    
    @Test
    public void testNormalizeHeadingError_180Degrees() {
        // 180° should stay as ±180° (implementation dependent)
        double error = Math.toRadians(180);
        double normalized = normalizeAngleSigned(error);
        assertEquals(Math.PI, Math.abs(normalized), 0.001);
    }
    
    // ===== HELPER METHODS =====
    
    /**
     * Select the closer hive based on distance
     */
    private Pose selectCloserHive(Pose robotPose, Pose leftHive, Pose rightHive) {
        double distLeft = distance(robotPose, leftHive);
        double distRight = distance(robotPose, rightHive);
        return distLeft < distRight ? leftHive : rightHive;
    }
    
    /**
     * Calculate distance between two poses
     */
    private double distance(Pose p1, Pose p2) {
        Vector2D v1 = p1.toVector2D();
        Vector2D v2 = p2.toVector2D();
        return v1.distance(v2);
    }
    
    /**
     * Calculate target heading to face a target pose
     */
    private double calculateTargetHeading(Pose robotPose, Pose targetPose) {
        Vector2D toTarget = targetPose.toVector2D().minus(robotPose.toVector2D());
        return toTarget.theta();
    }
    
    /**
     * Check if robot is aligned with target
     */
    private boolean isAligned(double currentHeading, double targetHeading, double toleranceDegrees) {
        double headingError = normalizeAngleSigned(targetHeading - currentHeading);
        return Math.abs(Math.toDegrees(headingError)) < toleranceDegrees;
    }
    
    /**
     * Normalize angle to [-π, π]
     */
    private double normalizeAngleSigned(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
    
    /**
     * Check if robot can fire
     */
    private boolean canFire(boolean isAligning, boolean aligned, boolean flywheelReady) {
        return isAligning && aligned && flywheelReady;
    }
    
    /**
     * Toggle intake state
     */
    private String toggleIntake(String currentState) {
        return currentState.equals("IDLE") ? "ON" : "IDLE";
    }
    
    /**
     * Save intake state when reversing
     */
    private String saveIntakeState(String currentState) {
        return currentState;
    }
    
    // ===== GAMEPAD INPUT MOCK TESTS =====
    
    @Test
    public void testGamepadEdgeDetection_RisingEdge() {
        // Test rising edge detection (button just pressed)
        gamepad.rightBumper = false;
        gamepad.previousRightBumper = false;
        
        // Press button
        gamepad.rightBumper = true;
        
        assertTrue("Should detect rising edge", 
            gamepad.rightBumper && !gamepad.previousRightBumper);
        
        // Update previous state
        gamepad.previousRightBumper = gamepad.rightBumper;
        
        // Button still held
        assertFalse("Should not detect rising edge while held",
            gamepad.rightBumper && !gamepad.previousRightBumper);
    }
    
    @Test
    public void testGamepadEdgeDetection_FallingEdge() {
        // Test falling edge detection (button just released)
        gamepad.leftBumper = true;
        gamepad.previousLeftBumper = true;
        
        // Release button
        gamepad.leftBumper = false;
        
        assertTrue("Should detect falling edge",
            !gamepad.leftBumper && gamepad.previousLeftBumper);
        
        // Update previous state
        gamepad.previousLeftBumper = gamepad.leftBumper;
        
        // Button still released
        assertFalse("Should not detect falling edge again",
            !gamepad.leftBumper && gamepad.previousLeftBumper);
    }
    
    @Test
    public void testGamepadTriggerThreshold() {
        // Test trigger threshold detection
        double TRIGGER_THRESHOLD = 0.1;
        
        gamepad.leftTrigger = 0.0;
        assertFalse("Zero trigger should be below threshold", gamepad.leftTrigger > TRIGGER_THRESHOLD);
        
        gamepad.leftTrigger = 0.05;
        assertFalse("Small trigger should be below threshold", gamepad.leftTrigger > TRIGGER_THRESHOLD);
        
        gamepad.leftTrigger = 0.1;
        assertFalse("Exact threshold should be below (exclusive)", gamepad.leftTrigger > TRIGGER_THRESHOLD);
        
        gamepad.leftTrigger = 0.11;
        assertTrue("Above threshold should be detected", gamepad.leftTrigger > TRIGGER_THRESHOLD);
        
        gamepad.leftTrigger = 1.0;
        assertTrue("Full trigger should be detected", gamepad.leftTrigger > TRIGGER_THRESHOLD);
    }
    
    // ===== INTAKE STATE MACHINE TESTS =====
    
    @Test
    public void testIntakeStateMachine_IdleToOnTransition() {
        robot.intakeState = IntakeState.IDLE;
        
        // Press right bumper
        gamepad.rightBumper = true;
        gamepad.previousRightBumper = false;
        
        // Process edge detection and toggle
        if (gamepad.rightBumper && !gamepad.previousRightBumper) {
            robot.intakeState = robot.intakeState == IntakeState.IDLE ? IntakeState.ON : IntakeState.IDLE;
        }
        
        assertEquals("Should transition to ON", IntakeState.ON, robot.intakeState);
    }
    
    @Test
    public void testIntakeStateMachine_OnToIdleTransition() {
        robot.intakeState = IntakeState.ON;
        
        // Press right bumper
        gamepad.rightBumper = true;
        gamepad.previousRightBumper = false;
        
        if (gamepad.rightBumper && !gamepad.previousRightBumper) {
            robot.intakeState = robot.intakeState == IntakeState.IDLE ? IntakeState.ON : IntakeState.IDLE;
        }
        
        assertEquals("Should transition to IDLE", IntakeState.IDLE, robot.intakeState);
    }
    
    @Test
    public void testIntakeStateMachine_ReversePreservesState() {
        robot.intakeState = IntakeState.ON;
        IntakeState savedState = robot.intakeState;
        
        // Press left bumper (reverse)
        gamepad.leftBumper = true;
        gamepad.previousLeftBumper = false;
        
        if (gamepad.leftBumper && !gamepad.previousLeftBumper) {
            savedState = robot.intakeState;
            robot.intakeReversing = true;
        }
        
        assertTrue("Should be reversing", robot.intakeReversing);
        assertEquals("Saved state should be ON", IntakeState.ON, savedState);
        
        // Release left bumper
        gamepad.leftBumper = false;
        gamepad.previousLeftBumper = true;
        
        if (!gamepad.leftBumper && gamepad.previousLeftBumper) {
            robot.intakeState = savedState;
            robot.intakeReversing = false;
        }
        
        assertFalse("Should stop reversing", robot.intakeReversing);
        assertEquals("Should restore saved state", IntakeState.ON, robot.intakeState);
    }
    
    @Test
    public void testIntakeStateMachine_ReverseFromIdle() {
        robot.intakeState = IntakeState.IDLE;
        IntakeState savedState = robot.intakeState;
        
        // Reverse from idle
        gamepad.leftBumper = true;
        gamepad.previousLeftBumper = false;
        
        if (gamepad.leftBumper && !gamepad.previousLeftBumper) {
            savedState = robot.intakeState;
            robot.intakeReversing = true;
        }
        
        assertTrue("Should be reversing", robot.intakeReversing);
        
        // Release
        gamepad.leftBumper = false;
        gamepad.previousLeftBumper = true;
        
        if (!gamepad.leftBumper && gamepad.previousLeftBumper) {
            robot.intakeState = savedState;
            robot.intakeReversing = false;
        }
        
        assertEquals("Should return to IDLE", IntakeState.IDLE, robot.intakeState);
    }
    
    @Test
    public void testIntakeStateMachine_ToggleWhileReversing() {
        robot.intakeState = IntakeState.ON;
        IntakeState savedState = robot.intakeState;
        
        // Start reversing
        gamepad.leftBumper = true;
        if (gamepad.leftBumper && !gamepad.previousLeftBumper) {
            savedState = robot.intakeState;
            robot.intakeReversing = true;
        }
        gamepad.previousLeftBumper = true;
        
        // Try to toggle with right bumper while reversing
        gamepad.rightBumper = true;
        gamepad.previousRightBumper = false;
        
        // Toggle should update saved state
        if (gamepad.rightBumper && !gamepad.previousRightBumper) {
            savedState = savedState == IntakeState.IDLE ? IntakeState.ON : IntakeState.IDLE;
        }
        
        assertEquals("Saved state should toggle", IntakeState.IDLE, savedState);
        
        // Release reverse
        gamepad.leftBumper = false;
        if (!gamepad.leftBumper && gamepad.previousLeftBumper) {
            robot.intakeState = savedState;
            robot.intakeReversing = false;
        }
        
        assertEquals("Should apply toggled saved state", IntakeState.IDLE, robot.intakeState);
    }
    
    // ===== FLYWHEEL STATE TRANSITION TESTS =====
    
    @Test
    public void testFlywheelStateTransition_AlignmentTriggered() {
        robot.flywheelOn = false;
        robot.flywheelVelocity = 0;
        
        // Align to target
        robot.aligned = true;
        robot.hasRumbled = false;
        
        // Process alignment
        if (robot.aligned && !robot.hasRumbled) {
            robot.flywheelOn = true;
            robot.hasRumbled = true;
        }
        
        assertTrue("Flywheel should start when aligned", robot.flywheelOn);
        assertTrue("Should mark as rumbled", robot.hasRumbled);
    }
    
    @Test
    public void testFlywheelStateTransition_LoseAlignment() {
        robot.flywheelOn = true;
        robot.aligned = true;
        
        // Lose alignment
        robot.aligned = false;
        
        // Process alignment check (FIX #1)
        if (robot.aligned) {
            robot.flywheelOn = true;
        } else {
            robot.flywheelOn = false;
        }
        
        assertFalse("Flywheel should stop when alignment lost", robot.flywheelOn);
    }
    
    @Test
    public void testFlywheelStateTransition_RegainAlignment() {
        robot.flywheelOn = false;
        robot.aligned = false;
        robot.hasRumbled = true; // Already rumbled before
        
        // Regain alignment
        robot.aligned = true;
        
        // Process alignment check
        if (robot.aligned) {
            robot.flywheelOn = true;
            // Don't rumble again if already rumbled
        }
        
        assertTrue("Flywheel should restart when aligned again", robot.flywheelOn);
    }
    
    @Test
    public void testFlywheelStateTransition_StopOnTriggerRelease() {
        robot.flywheelOn = true;
        
        // Release right trigger
        gamepad.rightTrigger = 0.0;
        gamepad.previousRightTrigger = 0.5;
        
        // Process trigger release
        if (gamepad.rightTrigger <= 0.1 && gamepad.previousRightTrigger > 0.1) {
            robot.flywheelOn = false;
        }
        
        assertFalse("Flywheel should stop when right trigger released", robot.flywheelOn);
    }
    
    @Test
    public void testFlywheelStateTransition_StopOnLeftTriggerRelease() {
        robot.flywheelOn = true;
        robot.isAligning = true;
        
        // Release left trigger
        gamepad.leftTrigger = 0.0;
        gamepad.previousLeftTrigger = 0.5;
        
        // Process trigger release
        if (gamepad.leftTrigger <= 0.1 && gamepad.previousLeftTrigger > 0.1) {
            robot.flywheelOn = false;
            robot.isAligning = false;
        }
        
        assertFalse("Flywheel should stop when left trigger released", robot.flywheelOn);
        assertFalse("Should exit aligning mode", robot.isAligning);
    }
    
    @Test
    public void testFlywheelVelocityRampUp() {
        robot.flywheelVelocity = 0;
        robot.flywheelOn = true;
        
        // Simulate velocity ramp up
        double targetVelocity = RobotHardware.FLYWHEEL_VELOCITY;
        double rampRate = 500.0; // ticks/sec per update
        
        // First update
        robot.flywheelVelocity = Math.min(robot.flywheelVelocity + rampRate, targetVelocity);
        assertEquals("Should ramp up", 500.0, robot.flywheelVelocity, 0.1);
        assertFalse("Should not be ready yet", 
            Math.abs(robot.flywheelVelocity - targetVelocity) < RobotHardware.FLYWHEEL_VELOCITY_TOLERANCE);
        
        // Continue ramping
        for (int i = 0; i < 10; i++) {
            robot.flywheelVelocity = Math.min(robot.flywheelVelocity + rampRate, targetVelocity);
        }
        
        assertTrue("Should reach target velocity", robot.flywheelVelocity >= targetVelocity);
        assertTrue("Should be ready",
            Math.abs(robot.flywheelVelocity - targetVelocity) < RobotHardware.FLYWHEEL_VELOCITY_TOLERANCE);
    }
    
    // ===== MOCK CLASSES =====
    
    private static class MockGamepadState {
        boolean rightBumper = false;
        boolean previousRightBumper = false;
        boolean leftBumper = false;
        boolean previousLeftBumper = false;
        double leftTrigger = 0.0;
        double previousLeftTrigger = 0.0;
        double rightTrigger = 0.0;
        double previousRightTrigger = 0.0;
    }
    
    private static class MockRobotState {
        IntakeState intakeState = IntakeState.IDLE;
        boolean intakeReversing = false;
        boolean flywheelOn = false;
        double flywheelVelocity = 0.0;
        boolean aligned = false;
        boolean hasRumbled = false;
        boolean isAligning = false;
    }
    
    private enum IntakeState {
        IDLE, ON
    }
}
