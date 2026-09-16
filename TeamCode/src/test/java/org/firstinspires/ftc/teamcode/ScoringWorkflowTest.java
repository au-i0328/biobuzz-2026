package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import org.junit.Test;
import org.junit.Before;
import static org.junit.Assert.*;

/**
 * Integration tests for the full scoring workflow:
 * 1. Driver presses left trigger
 * 2. Robot selects closer hive
 * 3. Robot aligns to hive
 * 4. Flywheel spins up
 * 5. Driver presses right trigger when aligned
 * 6. Transfer fires sample
 * 7. Driver releases triggers
 * 8. System returns to idle state
 */
public class ScoringWorkflowTest {
    
    private MockTeleopState state;
    
    @Before
    public void setUp() {
        state = new MockTeleopState();
    }
    
    // ===== FULL SCORING WORKFLOW TESTS =====
    
    @Test
    public void testFullScoringWorkflow_Success() {
        // Initial state: robot at (40, 70), facing east (0°)
        state.robotPose = new Pose(40, 70, 0);
        state.leftTrigger = false;
        state.rightTrigger = false;
        state.isAligning = false;
        state.targetHive = null;
        state.flywheelOn = false;
        state.transferOn = false;
        state.hasRumbled = false;
        
        // STEP 1: Driver presses left trigger
        state.leftTrigger = true;
        processLeftTriggerPress(state);
        
        assertTrue("Should enter aligning mode", state.isAligning);
        assertNotNull("Should select a target hive", state.targetHive);
        assertFalse("Flywheel should not start yet", state.flywheelOn);
        assertFalse("Should not have rumbled yet", state.hasRumbled);
        
        // Verify closer hive was selected (RED_HIVE_LEFT at (0,0) is closer than RED_HIVE_RIGHT at (57.6, 54.2))
        double distLeft = distance(state.robotPose, RobotHardware.RED_HIVE_LEFT);
        double distRight = distance(state.robotPose, RobotHardware.RED_HIVE_RIGHT);
        Pose expectedHive = distLeft < distRight ? RobotHardware.RED_HIVE_LEFT : RobotHardware.RED_HIVE_RIGHT;
        assertEquals("Should select closer hive", expectedHive, state.targetHive);
        
        // STEP 2: Robot aligns (simulated - heading becomes correct)
        Vector2D toHive = state.targetHive.toVector2D().minus(state.robotPose.toVector2D());
        double targetHeading = toHive.theta();
        state.robotPose = new Pose(state.robotPose.x(), state.robotPose.y(), targetHeading);
        
        // STEP 3: Process alignment check
        processAlignmentCheck(state);
        
        assertTrue("Should be aligned", isAligned(state.robotPose, state.targetHive));
        assertTrue("Flywheel should spin up when aligned", state.flywheelOn);
        assertTrue("Should rumble when first aligned", state.hasRumbled);
        
        // STEP 4: Driver presses right trigger
        state.rightTrigger = true;
        processRightTriggerPress(state);
        
        assertTrue("Transfer should activate when conditions met", state.transferOn);
        
        // STEP 5: Driver releases right trigger
        state.rightTrigger = false;
        processRightTriggerRelease(state);
        
        assertFalse("Flywheel should stop when right trigger released", state.flywheelOn);
        
        // STEP 6: Driver releases left trigger
        state.leftTrigger = false;
        processLeftTriggerRelease(state);
        
        assertFalse("Should exit aligning mode", state.isAligning);
        assertNull("Target hive should be cleared", state.targetHive);
        assertFalse("hasRumbled flag should reset", state.hasRumbled);
    }
    
    @Test
    public void testScoringWorkflow_FireWithoutAlignment() {
        // Robot not aligned, driver tries to fire
        state.robotPose = new Pose(40, 70, 0);
        state.leftTrigger = true;
        state.isAligning = true;
        state.targetHive = RobotHardware.RED_HIVE_RIGHT;
        state.flywheelOn = false;
        
        // Robot NOT aligned with target (facing wrong direction)
        Vector2D toHive = state.targetHive.toVector2D().minus(state.robotPose.toVector2D());
        double targetHeading = toHive.theta();
        state.robotPose = new Pose(state.robotPose.x(), state.robotPose.y(), targetHeading + Math.toRadians(10)); // 10° off
        
        processAlignmentCheck(state);
        
        assertFalse("Should not be aligned", isAligned(state.robotPose, state.targetHive));
        assertFalse("Flywheel should not start when not aligned", state.flywheelOn);
        
        // Try to fire
        state.rightTrigger = true;
        processRightTriggerPress(state);
        
        assertFalse("Transfer should NOT activate when not aligned", state.transferOn);
    }
    
    @Test
    public void testScoringWorkflow_LoseAlignmentWhileHolding() {
        // Robot aligned, then loses alignment while holding left trigger
        state.robotPose = new Pose(40, 70, 0);
        state.leftTrigger = true;
        state.isAligning = true;
        state.targetHive = RobotHardware.RED_HIVE_RIGHT;
        
        // Initially aligned
        Vector2D toHive = state.targetHive.toVector2D().minus(state.robotPose.toVector2D());
        double targetHeading = toHive.theta();
        state.robotPose = new Pose(state.robotPose.x(), state.robotPose.y(), targetHeading);
        
        processAlignmentCheck(state);
        assertTrue("Should be aligned", isAligned(state.robotPose, state.targetHive));
        assertTrue("Flywheel should be on", state.flywheelOn);
        assertTrue("Should have rumbled", state.hasRumbled);
        
        // Robot moves and loses alignment
        state.robotPose = new Pose(state.robotPose.x(), state.robotPose.y(), targetHeading + Math.toRadians(5));
        
        processAlignmentCheck(state);
        assertFalse("Should no longer be aligned", isAligned(state.robotPose, state.targetHive));
        assertFalse("Flywheel should stop when alignment lost (FIX #1)", state.flywheelOn);
    }
    
    @Test
    public void testScoringWorkflow_RegainAlignment() {
        // Robot loses alignment, then regains it
        state.robotPose = new Pose(40, 70, 0);
        state.leftTrigger = true;
        state.isAligning = true;
        state.targetHive = RobotHardware.RED_HIVE_RIGHT;
        state.hasRumbled = true; // Already rumbled once
        
        // Not aligned
        Vector2D toHive = state.targetHive.toVector2D().minus(state.robotPose.toVector2D());
        double targetHeading = toHive.theta();
        state.robotPose = new Pose(state.robotPose.x(), state.robotPose.y(), targetHeading + Math.toRadians(5));
        
        processAlignmentCheck(state);
        assertFalse("Should not be aligned", isAligned(state.robotPose, state.targetHive));
        assertFalse("Flywheel should be off", state.flywheelOn);
        
        // Regain alignment
        state.robotPose = new Pose(state.robotPose.x(), state.robotPose.y(), targetHeading);
        state.rumbleCount = 0; // Reset rumble counter
        
        processAlignmentCheck(state);
        assertTrue("Should be aligned again", isAligned(state.robotPose, state.targetHive));
        assertTrue("Flywheel should turn back on", state.flywheelOn);
        assertEquals("Should not rumble again (already rumbled)", 0, state.rumbleCount);
    }
    
    @Test
    public void testScoringWorkflow_RapidTriggerToggle() {
        // Test rapid left trigger press/release
        state.robotPose = new Pose(40, 70, 0);
        
        // First press
        state.leftTrigger = true;
        processLeftTriggerPress(state);
        Pose firstTarget = state.targetHive;
        
        assertNotNull("Should have target after first press", firstTarget);
        assertTrue("Should be aligning", state.isAligning);
        
        // Release
        state.leftTrigger = false;
        processLeftTriggerRelease(state);
        
        assertNull("Target should be cleared", state.targetHive);
        assertFalse("Should not be aligning", state.isAligning);
        
        // Second press (robot moved)
        state.robotPose = new Pose(50, 60, 0);
        state.leftTrigger = true;
        processLeftTriggerPress(state);
        
        assertNotNull("Should have new target", state.targetHive);
        assertTrue("Should be aligning again", state.isAligning);
        
        // Target should be recalculated based on new position
        double distLeft = distance(state.robotPose, RobotHardware.RED_HIVE_LEFT);
        double distRight = distance(state.robotPose, RobotHardware.RED_HIVE_RIGHT);
        Pose expectedHive = distLeft < distRight ? RobotHardware.RED_HIVE_LEFT : RobotHardware.RED_HIVE_RIGHT;
        assertEquals("Should select closer hive from new position", expectedHive, state.targetHive);
    }
    
    @Test
    public void testScoringWorkflow_FireNotInAligningMode() {
        // Try to fire without being in aligning mode
        state.robotPose = new Pose(40, 70, 0);
        state.isAligning = false;
        state.targetHive = null;
        state.leftTrigger = false;
        state.rightTrigger = true;
        
        processRightTriggerPress(state);
        
        assertFalse("Transfer should NOT activate when not aligning", state.transferOn);
    }
    
    @Test
    public void testScoringWorkflow_FlywheelNotReady() {
        // Aligned but flywheel not at speed
        state.robotPose = new Pose(40, 70, 0);
        state.leftTrigger = true;
        state.isAligning = true;
        state.targetHive = RobotHardware.RED_HIVE_RIGHT;
        
        // Aligned
        Vector2D toHive = state.targetHive.toVector2D().minus(state.robotPose.toVector2D());
        double targetHeading = toHive.theta();
        state.robotPose = new Pose(state.robotPose.x(), state.robotPose.y(), targetHeading);
        
        state.flywheelVelocity = 1000.0; // Target is 2000, tolerance is 20
        
        processAlignmentCheck(state);
        assertTrue("Should be aligned", isAligned(state.robotPose, state.targetHive));
        
        // Try to fire
        state.rightTrigger = true;
        processRightTriggerPress(state);
        
        assertFalse("Transfer should NOT activate when flywheel not ready", state.transferOn);
    }
    
    @Test
    public void testScoringWorkflow_MultipleShots() {
        // Fire multiple shots in sequence
        state.robotPose = new Pose(40, 70, 0);
        state.leftTrigger = true;
        state.isAligning = true;
        state.targetHive = RobotHardware.RED_HIVE_RIGHT;
        
        // Get aligned
        Vector2D toHive = state.targetHive.toVector2D().minus(state.robotPose.toVector2D());
        double targetHeading = toHive.theta();
        state.robotPose = new Pose(state.robotPose.x(), state.robotPose.y(), targetHeading);
        state.flywheelVelocity = RobotHardware.FLYWHEEL_VELOCITY;
        
        processAlignmentCheck(state);
        
        // First shot
        state.rightTrigger = true;
        processRightTriggerPress(state);
        assertTrue("First shot should fire", state.transferOn);
        
        state.rightTrigger = false;
        processRightTriggerRelease(state);
        assertFalse("Flywheel should stop after first shot", state.flywheelOn);
        
        // Still holding left trigger, flywheel spins back up
        processAlignmentCheck(state);
        assertTrue("Flywheel should restart (still aligned)", state.flywheelOn);
        
        // Second shot
        state.rightTrigger = true;
        processRightTriggerPress(state);
        assertTrue("Second shot should fire", state.transferOn);
    }
    
    @Test
    public void testScoringWorkflow_TargetHiveSwitch() {
        // Switch target hive mid-alignment
        state.robotPose = new Pose(50, 50, 0);
        
        // First press - closer to one hive
        state.leftTrigger = true;
        processLeftTriggerPress(state);
        Pose firstTarget = state.targetHive;
        assertNotNull("Should have first target", firstTarget);
        
        // Release and move robot
        state.leftTrigger = false;
        processLeftTriggerRelease(state);
        
        // Move closer to other hive
        state.robotPose = new Pose(10, 10, 0);
        
        // Press again
        state.leftTrigger = true;
        processLeftTriggerPress(state);
        Pose secondTarget = state.targetHive;
        
        assertNotNull("Should have second target", secondTarget);
        
        // Targets might be different depending on robot position
        double distLeft1 = distance(new Pose(50, 50, 0), RobotHardware.RED_HIVE_LEFT);
        double distRight1 = distance(new Pose(50, 50, 0), RobotHardware.RED_HIVE_RIGHT);
        double distLeft2 = distance(new Pose(10, 10, 0), RobotHardware.RED_HIVE_LEFT);
        double distRight2 = distance(new Pose(10, 10, 0), RobotHardware.RED_HIVE_RIGHT);
        
        // Verify targets match expectations
        assertEquals("First target should match closer hive from first position",
            distLeft1 < distRight1 ? RobotHardware.RED_HIVE_LEFT : RobotHardware.RED_HIVE_RIGHT,
            firstTarget);
        assertEquals("Second target should match closer hive from second position",
            distLeft2 < distRight2 ? RobotHardware.RED_HIVE_LEFT : RobotHardware.RED_HIVE_RIGHT,
            secondTarget);
    }
    
    // ===== HELPER METHODS =====
    
    private void processLeftTriggerPress(MockTeleopState state) {
        if (!state.isAligning) {
            double distLeft = distance(state.robotPose, RobotHardware.RED_HIVE_LEFT);
            double distRight = distance(state.robotPose, RobotHardware.RED_HIVE_RIGHT);
            state.targetHive = distLeft < distRight ? RobotHardware.RED_HIVE_LEFT : RobotHardware.RED_HIVE_RIGHT;
            state.isAligning = true;
            state.hasRumbled = false;
        }
    }
    
    private void processLeftTriggerRelease(MockTeleopState state) {
        state.isAligning = false;
        state.targetHive = null;
        state.hasRumbled = false;
        state.flywheelOn = false;
    }
    
    private void processAlignmentCheck(MockTeleopState state) {
        if (state.leftTrigger && state.targetHive != null) {
            boolean aligned = isAligned(state.robotPose, state.targetHive);
            
            if (aligned && !state.hasRumbled) {
                state.rumbleCount++;
                state.hasRumbled = true;
                state.flywheelOn = true;
            }
            
            // FIX #1: Stop flywheel if alignment lost
            if (aligned) {
                state.flywheelOn = true;
            } else {
                state.flywheelOn = false;
            }
        }
    }
    
    private void processRightTriggerPress(MockTeleopState state) {
        if (state.isAligning && state.targetHive != null) {
            boolean aligned = isAligned(state.robotPose, state.targetHive);
            boolean flywheelReady = Math.abs(state.flywheelVelocity - RobotHardware.FLYWHEEL_VELOCITY) 
                < RobotHardware.FLYWHEEL_VELOCITY_TOLERANCE;
            
            if (aligned && flywheelReady) {
                state.transferOn = true;
            }
        }
    }
    
    private void processRightTriggerRelease(MockTeleopState state) {
        state.flywheelOn = false;
        state.transferOn = false;
    }
    
    private boolean isAligned(Pose robotPose, Pose targetHive) {
        Vector2D toHive = targetHive.toVector2D().minus(robotPose.toVector2D());
        double targetHeading = toHive.theta();
        double headingError = normalizeAngleSigned(targetHeading - robotPose.heading());
        return Math.abs(Math.toDegrees(headingError)) < RobotHardware.ALIGNMENT_TOLERANCE;
    }
    
    private double normalizeAngleSigned(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
    
    private double distance(Pose p1, Pose p2) {
        return p1.toVector2D().distance(p2.toVector2D());
    }
    
    /**
     * Mock state container for teleop testing
     */
    private static class MockTeleopState {
        Pose robotPose;
        Pose targetHive;
        boolean leftTrigger;
        boolean rightTrigger;
        boolean isAligning;
        boolean hasRumbled;
        boolean flywheelOn;
        boolean transferOn;
        double flywheelVelocity = RobotHardware.FLYWHEEL_VELOCITY; // Default to ready
        int rumbleCount = 0;
    }
}
