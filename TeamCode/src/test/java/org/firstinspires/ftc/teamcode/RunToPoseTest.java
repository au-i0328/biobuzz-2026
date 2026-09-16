package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import org.junit.Test;
import org.junit.Before;
import static org.junit.Assert.*;

/**
 * Unit tests for RunToPose collision detection and geometry methods.
 * 
 * Note: Pathfinding and full goToPose() require hardware mocks and are
 * tested separately in integration tests.
 */
public class RunToPoseTest {
    
    private TestableRunToPose runToPose;
    
    @Before
    public void setUp() {
        runToPose = new TestableRunToPose();
    }
    
    // ===== Line-Rectangle Intersection Tests =====
    
    @Test
    public void testLineIntersectsRect_StartInsideRect() {
        // Line starts inside rectangle
        boolean result = runToPose.testLineIntersectsRect(
            5, 5,  // start (inside)
            15, 15, // end (outside)
            0, 10,  // rect X bounds
            0, 10   // rect Y bounds
        );
        assertTrue("Line starting inside rect should intersect", result);
    }
    
    @Test
    public void testLineIntersectsRect_EndInsideRect() {
        // Line ends inside rectangle
        boolean result = runToPose.testLineIntersectsRect(
            15, 15, // start (outside)
            5, 5,   // end (inside)
            0, 10,  // rect X bounds
            0, 10   // rect Y bounds
        );
        assertTrue("Line ending inside rect should intersect", result);
    }
    
    @Test
    public void testLineIntersectsRect_CrossesThrough() {
        // Line crosses through rectangle
        boolean result = runToPose.testLineIntersectsRect(
            -5, 5,  // start (left of rect)
            15, 5,  // end (right of rect)
            0, 10,  // rect X bounds
            0, 10   // rect Y bounds
        );
        assertTrue("Line crossing through rect should intersect", result);
    }
    
    @Test
    public void testLineIntersectsRect_NoIntersection() {
        // Line completely outside rectangle
        boolean result = runToPose.testLineIntersectsRect(
            -5, -5,  // start
            -3, -3,  // end
            0, 10,   // rect X bounds
            0, 10    // rect Y bounds
        );
        assertFalse("Line outside rect should not intersect", result);
    }
    
    @Test
    public void testLineIntersectsRect_TouchesCorner() {
        // Line passes through corner
        boolean result = runToPose.testLineIntersectsRect(
            -5, -5, // start
            15, 15, // end (diagonal through 0,0 corner)
            0, 10,  // rect X bounds
            0, 10   // rect Y bounds
        );
        assertTrue("Line through corner should intersect", result);
    }
    
    // ===== Line-Line Intersection Tests =====
    
    @Test
    public void testLineIntersectsLine_PerpendicularCross() {
        // Two perpendicular lines crossing
        boolean result = runToPose.testLineIntersectsLine(
            0, 5,   // line 1 start
            10, 5,  // line 1 end (horizontal)
            5, 0,   // line 2 start
            5, 10   // line 2 end (vertical)
        );
        assertTrue("Perpendicular crossing lines should intersect", result);
    }
    
    @Test
    public void testLineIntersectsLine_Parallel() {
        // Parallel lines
        boolean result = runToPose.testLineIntersectsLine(
            0, 0,   // line 1 start
            10, 0,  // line 1 end (horizontal)
            0, 5,   // line 2 start
            10, 5   // line 2 end (horizontal, parallel)
        );
        assertFalse("Parallel lines should not intersect", result);
    }
    
    @Test
    public void testLineIntersectsLine_NoIntersection() {
        // Lines that don't intersect
        boolean result = runToPose.testLineIntersectsLine(
            0, 0,   // line 1 start
            5, 0,   // line 1 end
            10, 10, // line 2 start
            15, 15  // line 2 end
        );
        assertFalse("Non-intersecting lines should not intersect", result);
    }
    
    @Test
    public void testLineIntersectsLine_TouchingEndpoints() {
        // Lines sharing an endpoint
        boolean result = runToPose.testLineIntersectsLine(
            0, 0,   // line 1 start
            5, 5,   // line 1 end
            5, 5,   // line 2 start (same as line 1 end)
            10, 10  // line 2 end
        );
        assertTrue("Lines sharing endpoint should intersect", result);
    }
    
    // ===== Forbidden Zone Tests =====
    
    @Test
    public void testIsInRedStructure_Inside() {
        boolean result = runToPose.testIsInRedStructure(47.5, 70);
        assertTrue("Point inside red structure should be detected", result);
    }
    
    @Test
    public void testIsInRedStructure_Outside() {
        boolean result = runToPose.testIsInRedStructure(30, 70);
        assertFalse("Point outside red structure should not be detected", result);
    }
    
    @Test
    public void testIsInBlueStructure_Inside() {
        boolean result = runToPose.testIsInBlueStructure(94.5, 70);
        assertTrue("Point inside blue structure should be detected", result);
    }
    
    @Test
    public void testIsInBlueStructure_Outside() {
        boolean result = runToPose.testIsInBlueStructure(110, 70);
        assertFalse("Point outside blue structure should not be detected", result);
    }
    
    @Test
    public void testIsInForbiddenZone_InRed() {
        boolean result = runToPose.testIsInForbiddenZone(47.5, 70);
        assertTrue("Point in red structure should be in forbidden zone", result);
    }
    
    @Test
    public void testIsInForbiddenZone_InBlue() {
        boolean result = runToPose.testIsInForbiddenZone(94.5, 70);
        assertTrue("Point in blue structure should be in forbidden zone", result);
    }
    
    @Test
    public void testIsInForbiddenZone_Outside() {
        boolean result = runToPose.testIsInForbiddenZone(72, 72);
        assertFalse("Point outside both structures should not be in forbidden zone", result);
    }
    
    // ===== Path Crossing Forbidden Zone Tests =====
    
    @Test
    public void testPathCrossesForbiddenZone_CrossesRed() {
        Pose start = new Pose(30, 70, 0);
        Pose end = new Pose(60, 70, 0);
        boolean result = runToPose.testPathCrossesForbiddenZone(start, end);
        assertTrue("Path crossing red structure should be detected", result);
    }
    
    @Test
    public void testPathCrossesForbiddenZone_CrossesBlue() {
        Pose start = new Pose(85, 70, 0);
        Pose end = new Pose(105, 70, 0);
        boolean result = runToPose.testPathCrossesForbiddenZone(start, end);
        assertTrue("Path crossing blue structure should be detected", result);
    }
    
    @Test
    public void testPathCrossesForbiddenZone_NoCrossing() {
        Pose start = new Pose(20, 20, 0);
        Pose end = new Pose(30, 30, 0);
        boolean result = runToPose.testPathCrossesForbiddenZone(start, end);
        assertFalse("Path not crossing any structure should not be detected", result);
    }
    
    @Test
    public void testPathCrossesForbiddenZone_BetweenStructures() {
        Pose start = new Pose(60, 70, 0);
        Pose end = new Pose(85, 70, 0);
        boolean result = runToPose.testPathCrossesForbiddenZone(start, end);
        assertFalse("Path between structures should not cross", result);
    }
    
    // ===== Angle Normalization Tests =====
    
    @Test
    public void testNormalizeAngle_Positive() {
        double result = runToPose.testNormalizeAngle(Math.PI * 1.5);
        assertEquals(-Math.PI / 2, result, 0.0001);
    }
    
    @Test
    public void testNormalizeAngle_Negative() {
        double result = runToPose.testNormalizeAngle(-Math.PI * 1.5);
        assertEquals(Math.PI / 2, result, 0.0001);
    }
    
    @Test
    public void testNormalizeAngle_AlreadyNormalized() {
        double result = runToPose.testNormalizeAngle(Math.PI / 4);
        assertEquals(Math.PI / 4, result, 0.0001);
    }
    
    @Test
    public void testNormalizeAngle_FullRotation() {
        double result = runToPose.testNormalizeAngle(2 * Math.PI);
        assertEquals(0, result, 0.0001);
    }
    
    // ===== Shortest Angle Tests =====
    
    @Test
    public void testCalculateShortestAngle_ShortPath() {
        double current = Math.toRadians(10);
        double target = Math.toRadians(50);
        double result = runToPose.testCalculateShortestAngle(current, target);
        assertEquals(Math.toRadians(50), result, 0.0001);
    }
    
    @Test
    public void testCalculateShortestAngle_WrapAround() {
        // From 170° to -170° should go via 180° (shortest is -20° turn)
        double current = Math.toRadians(170);
        double target = Math.toRadians(-170);
        double result = runToPose.testCalculateShortestAngle(current, target);
        // Result should be target + 360° to indicate clockwise wrap
        assertTrue("Should wrap around", Math.abs(result - Math.toRadians(190)) < 0.0001);
    }
    
    @Test
    public void testCalculateShortestAngle_OppositeWrapAround() {
        // From -170° to 170° should go via -180° (shortest is 20° turn)
        double current = Math.toRadians(-170);
        double target = Math.toRadians(170);
        double result = runToPose.testCalculateShortestAngle(current, target);
        // Result should be target - 360° to indicate counter-clockwise wrap
        assertTrue("Should wrap around opposite direction", Math.abs(result - Math.toRadians(-190)) < 0.0001);
    }
    
    /**
     * Testable subclass that exposes private methods for unit testing
     */
    private static class TestableRunToPose {
        
        private static final double SAFETY_MARGIN = 10.0;
        private static final double RED_STRUCTURE_MIN_X = 45 - SAFETY_MARGIN;
        private static final double RED_STRUCTURE_MAX_X = 50 + SAFETY_MARGIN;
        private static final double RED_STRUCTURE_MIN_Y = 48 - SAFETY_MARGIN;
        private static final double RED_STRUCTURE_MAX_Y = 93 + SAFETY_MARGIN;
        
        private static final double BLUE_STRUCTURE_MIN_X = 92 - SAFETY_MARGIN;
        private static final double BLUE_STRUCTURE_MAX_X = 97 + SAFETY_MARGIN;
        private static final double BLUE_STRUCTURE_MIN_Y = 48 - SAFETY_MARGIN;
        private static final double BLUE_STRUCTURE_MAX_Y = 93 + SAFETY_MARGIN;
        
        public boolean testLineIntersectsRect(double x1, double y1, double x2, double y2,
                                             double rectMinX, double rectMaxX,
                                             double rectMinY, double rectMaxY) {
            if ((x1 >= rectMinX && x1 <= rectMaxX && y1 >= rectMinY && y1 <= rectMaxY) ||
                (x2 >= rectMinX && x2 <= rectMaxX && y2 >= rectMinY && y2 <= rectMaxY)) {
                return true;
            }
            
            if (testLineIntersectsLine(x1, y1, x2, y2, rectMinX, rectMinY, rectMinX, rectMaxY)) return true;
            if (testLineIntersectsLine(x1, y1, x2, y2, rectMaxX, rectMinY, rectMaxX, rectMaxY)) return true;
            if (testLineIntersectsLine(x1, y1, x2, y2, rectMinX, rectMinY, rectMaxX, rectMinY)) return true;
            if (testLineIntersectsLine(x1, y1, x2, y2, rectMinX, rectMaxY, rectMaxX, rectMaxY)) return true;
            
            return false;
        }
        
        public boolean testLineIntersectsLine(double x1, double y1, double x2, double y2,
                                             double x3, double y3, double x4, double y4) {
            double denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
            if (Math.abs(denom) < 1e-10) return false;
            
            double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom;
            double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom;
            
            return t >= 0 && t <= 1 && u >= 0 && u <= 1;
        }
        
        public boolean testIsInRedStructure(double x, double y) {
            return x >= RED_STRUCTURE_MIN_X && x <= RED_STRUCTURE_MAX_X &&
                   y >= RED_STRUCTURE_MIN_Y && y <= RED_STRUCTURE_MAX_Y;
        }
        
        public boolean testIsInBlueStructure(double x, double y) {
            return x >= BLUE_STRUCTURE_MIN_X && x <= BLUE_STRUCTURE_MAX_X &&
                   y >= BLUE_STRUCTURE_MIN_Y && y <= BLUE_STRUCTURE_MAX_Y;
        }
        
        public boolean testIsInForbiddenZone(double x, double y) {
            return testIsInRedStructure(x, y) || testIsInBlueStructure(x, y);
        }
        
        public boolean testPathCrossesForbiddenZone(Pose current, Pose target) {
            return testLineIntersectsRect(current.x(), current.y(), target.x(), target.y(),
                                         RED_STRUCTURE_MIN_X, RED_STRUCTURE_MAX_X,
                                         RED_STRUCTURE_MIN_Y, RED_STRUCTURE_MAX_Y) ||
                   testLineIntersectsRect(current.x(), current.y(), target.x(), target.y(),
                                         BLUE_STRUCTURE_MIN_X, BLUE_STRUCTURE_MAX_X,
                                         BLUE_STRUCTURE_MIN_Y, BLUE_STRUCTURE_MAX_Y);
        }
        
        public double testNormalizeAngle(double angle) {
            while (angle > Math.PI) angle -= 2 * Math.PI;
            while (angle < -Math.PI) angle += 2 * Math.PI;
            return angle;
        }
        
        public double testCalculateShortestAngle(double currentHeading, double targetHeading) {
            double current = testNormalizeAngle(currentHeading);
            double target = testNormalizeAngle(targetHeading);
            
            double diff = target - current;
            
            if (diff > Math.PI) {
                return target - 2 * Math.PI;
            } else if (diff < -Math.PI) {
                return target + 2 * Math.PI;
            } else {
                return target;
            }
        }
    }
}
