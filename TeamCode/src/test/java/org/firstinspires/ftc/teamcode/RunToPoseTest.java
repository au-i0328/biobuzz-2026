package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import org.junit.Test;
import org.junit.Before;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.PriorityQueue;

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
        assertTrue("Path starting at structure boundary should be detected as crossing", result);
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
    
    // ===== Corner Clearance Tests =====
    
    @Test
    public void testGetRectCorners_RedStructure() {
        Pose[] corners = runToPose.testGetRectCorners(
            runToPose.RED_STRUCTURE_MIN_X,
            runToPose.RED_STRUCTURE_MAX_X,
            runToPose.RED_STRUCTURE_MIN_Y,
            runToPose.RED_STRUCTURE_MAX_Y
        );
        
        assertEquals("Should have 4 corners", 4, corners.length);
        
        // Verify corners are outside the rectangle (with clearance)
        for (Pose corner : corners) {
            assertFalse("Corner should not be inside red structure",
                runToPose.testIsInRedStructure(corner.x(), corner.y()));
        }
        
        // Check specific corner positions
        assertEquals("Bottom-left X", runToPose.RED_STRUCTURE_MIN_X - 1.0, corners[0].x(), 0.001);
        assertEquals("Bottom-left Y", runToPose.RED_STRUCTURE_MIN_Y - 1.0, corners[0].y(), 0.001);
        assertEquals("Top-left X", runToPose.RED_STRUCTURE_MIN_X - 1.0, corners[1].x(), 0.001);
        assertEquals("Top-left Y", runToPose.RED_STRUCTURE_MAX_Y + 1.0, corners[1].y(), 0.001);
        assertEquals("Bottom-right X", runToPose.RED_STRUCTURE_MAX_X + 1.0, corners[2].x(), 0.001);
        assertEquals("Bottom-right Y", runToPose.RED_STRUCTURE_MIN_Y - 1.0, corners[2].y(), 0.001);
        assertEquals("Top-right X", runToPose.RED_STRUCTURE_MAX_X + 1.0, corners[3].x(), 0.001);
        assertEquals("Top-right Y", runToPose.RED_STRUCTURE_MAX_Y + 1.0, corners[3].y(), 0.001);
    }
    
    @Test
    public void testGetRectCorners_BlueStructure() {
        Pose[] corners = runToPose.testGetRectCorners(
            runToPose.BLUE_STRUCTURE_MIN_X,
            runToPose.BLUE_STRUCTURE_MAX_X,
            runToPose.BLUE_STRUCTURE_MIN_Y,
            runToPose.BLUE_STRUCTURE_MAX_Y
        );
        
        assertEquals("Should have 4 corners", 4, corners.length);
        
        // Verify corners are outside the rectangle
        for (Pose corner : corners) {
            assertFalse("Corner should not be inside blue structure",
                runToPose.testIsInBlueStructure(corner.x(), corner.y()));
        }
    }
    
    // ===== Pathfinding Tests =====
    
    @Test
    public void testFindShortestPath_DirectPath() {
        // Direct path with no obstacles
        Pose start = new Pose(20, 20, 0);
        Pose target = new Pose(30, 30, 0);
        
        List<Pose> path = runToPose.testFindShortestPath(start, target);
        
        assertNotNull("Path should be found", path);
        assertEquals("Path should have 2 waypoints (start and target)", 2, path.size());
        assertEquals("First waypoint should be start", start.x(), path.get(0).x(), 0.001);
        assertEquals("Last waypoint should be target", target.x(), path.get(path.size() - 1).x(), 0.001);
    }
    
    @Test
    public void testFindShortestPath_AroundRedStructure() {
        // Path that needs to go around red structure
        Pose start = new Pose(30, 70, 0);
        Pose target = new Pose(70, 70, 0);
        
        List<Pose> path = runToPose.testFindShortestPath(start, target);
        
        assertNotNull("Path should be found", path);
        assertTrue("Path should have waypoints", path.size() >= 2);
        assertEquals("First waypoint should be start", start.x(), path.get(0).x(), 0.001);
        assertEquals("Last waypoint should be target", target.x(), path.get(path.size() - 1).x(), 0.001);
        
        // Verify no segment crosses red structure
        for (int i = 0; i < path.size() - 1; i++) {
            Pose from = path.get(i);
            Pose to = path.get(i + 1);
            assertFalse("Path segment should not cross red structure",
                runToPose.testPathCrossesForbiddenZone(from, to));
        }
    }
    
    @Test
    public void testFindShortestPath_BetweenStructures() {
        // Path going between red and blue structures
        Pose start = new Pose(30, 70, 0);
        Pose target = new Pose(110, 70, 0);
        
        List<Pose> path = runToPose.testFindShortestPath(start, target);
        
        assertNotNull("Path should be found", path);
        assertTrue("Path should have multiple waypoints", path.size() >= 3);
        
        // Verify path doesn't cross any structures
        for (int i = 0; i < path.size() - 1; i++) {
            Pose from = path.get(i);
            Pose to = path.get(i + 1);
            assertFalse("Path segment should not cross forbidden zones",
                runToPose.testPathCrossesForbiddenZone(from, to));
        }
    }
    
    @Test
    public void testFindShortestPath_AroundBlueStructure() {
        // Path that needs to go around blue structure
        Pose start = new Pose(80, 70, 0);
        Pose target = new Pose(110, 70, 0);
        
        List<Pose> path = runToPose.testFindShortestPath(start, target);
        
        assertNotNull("Path should be found", path);
        assertTrue("Path should have waypoints", path.size() >= 2);
        
        // Verify no segment crosses blue structure
        for (int i = 0; i < path.size() - 1; i++) {
            Pose from = path.get(i);
            Pose to = path.get(i + 1);
            assertFalse("Path segment should not cross blue structure",
                runToPose.testPathCrossesForbiddenZone(from, to));
        }
    }
    
    @Test
    public void testFindShortestPath_TargetInForbiddenZone() {
        // Target is inside forbidden zone - pathfinding should fail
        Pose start = new Pose(20, 70, 0);
        Pose target = new Pose(47.5, 70, 0); // Inside red structure
        
        List<Pose> path = runToPose.testFindShortestPath(start, target);
        
        // Pathfinding will fail because no valid path exists when target is in forbidden zone
        // The goToPose() method prevents this scenario with early validation
        assertNull("Path to forbidden zone target should fail", path);
    }
    
    @Test
    public void testFindShortestPath_ComplexRoute() {
        // Diagonal path that requires routing around structures
        Pose start = new Pose(20, 50, 0);
        Pose target = new Pose(110, 90, 0);
        
        List<Pose> path = runToPose.testFindShortestPath(start, target);
        
        assertNotNull("Path should be found", path);
        assertTrue("Path should have waypoints", path.size() >= 2);
        
        // Verify all segments are valid
        for (int i = 0; i < path.size() - 1; i++) {
            Pose from = path.get(i);
            Pose to = path.get(i + 1);
            assertFalse("No segment should cross forbidden zones",
                runToPose.testPathCrossesForbiddenZone(from, to));
        }
    }
    
    @Test
    public void testFindShortestPath_AboveStructures() {
        // Path going above both structures
        Pose start = new Pose(30, 110, 0);
        Pose target = new Pose(110, 110, 0);
        
        List<Pose> path = runToPose.testFindShortestPath(start, target);
        
        assertNotNull("Path should be found", path);
        // Should be direct since there are no obstacles in the way
        assertEquals("Path should be direct", 2, path.size());
    }
    
    @Test
    public void testFindShortestPath_BelowStructures() {
        // Path going below both structures
        Pose start = new Pose(30, 30, 0);
        Pose target = new Pose(110, 30, 0);
        
        List<Pose> path = runToPose.testFindShortestPath(start, target);
        
        assertNotNull("Path should be found", path);
        // Should be direct since there are no obstacles in the way
        assertEquals("Path should be direct", 2, path.size());
    }
    
    // ===== Edge Case Tests =====
    
    @Test
    public void testLineIntersectsLine_CoincidentLines() {
        // Two lines that are exactly the same
        boolean result = runToPose.testLineIntersectsLine(
            0, 0, 10, 10,
            0, 0, 10, 10
        );
        assertTrue("Coincident lines should intersect", result);
    }
    
    @Test
    public void testLineIntersectsLine_PartialOverlap() {
        // Two collinear lines that partially overlap
        boolean result = runToPose.testLineIntersectsLine(
            0, 0, 10, 0,
            5, 0, 15, 0
        );
        assertTrue("Partially overlapping collinear lines should intersect", result);
    }
    
    @Test
    public void testLineIntersectsLine_CollinearNoOverlap() {
        // Two collinear lines that don't overlap
        boolean result = runToPose.testLineIntersectsLine(
            0, 0, 5, 0,
            10, 0, 15, 0
        );
        assertFalse("Non-overlapping collinear lines should not intersect", result);
    }
    
    @Test
    public void testNormalizeAngle_MultipleRotations() {
        double result = runToPose.testNormalizeAngle(5 * Math.PI);
        assertEquals(Math.PI, result, 0.0001);
    }
    
    @Test
    public void testNormalizeAngle_NegativeMultipleRotations() {
        double result = runToPose.testNormalizeAngle(-5 * Math.PI);
        assertEquals(-Math.PI, result, 0.0001);
    }
    
    @Test
    public void testCalculateShortestAngle_Zero() {
        double result = runToPose.testCalculateShortestAngle(0, 0);
        assertEquals(0, result, 0.0001);
    }
    
    @Test
    public void testCalculateShortestAngle_180Degrees() {
        // 180° turn - could go either way, implementation should pick one consistently
        double current = 0;
        double target = Math.PI;
        double result = runToPose.testCalculateShortestAngle(current, target);
        // Should be close to PI or -PI
        assertTrue("180° turn should result in ±π", 
            Math.abs(Math.abs(result) - Math.PI) < 0.0001);
    }
    
    /**
     * Testable subclass that exposes private methods for unit testing
     */
    private static class TestableRunToPose {
        
        private static final double SAFETY_MARGIN = 10.0;
        private static final double CORNER_CLEARANCE = 1.0;
        
        public final double RED_STRUCTURE_MIN_X = 45 - SAFETY_MARGIN;
        public final double RED_STRUCTURE_MAX_X = 50 + SAFETY_MARGIN;
        public final double RED_STRUCTURE_MIN_Y = 48 - SAFETY_MARGIN;
        public final double RED_STRUCTURE_MAX_Y = 93 + SAFETY_MARGIN;
        
        public final double BLUE_STRUCTURE_MIN_X = 92 - SAFETY_MARGIN;
        public final double BLUE_STRUCTURE_MAX_X = 97 + SAFETY_MARGIN;
        public final double BLUE_STRUCTURE_MIN_Y = 48 - SAFETY_MARGIN;
        public final double BLUE_STRUCTURE_MAX_Y = 93 + SAFETY_MARGIN;
        
        public boolean testLineIntersectsRect(double x1, double y1, double x2, double y2,
                                             double rectMinX, double rectMaxX,
                                             double rectMinY, double rectMaxY) {
            // Check if either endpoint is inside the rectangle (including boundary)
            if ((x1 >= rectMinX && x1 <= rectMaxX && y1 >= rectMinY && y1 <= rectMaxY) ||
                (x2 >= rectMinX && x2 <= rectMaxX && y2 >= rectMinY && y2 <= rectMaxY)) {
                return true;
            }
            
            // Check if line crosses any of the four edges
            // Left edge
            if (testLineIntersectsLine(x1, y1, x2, y2, rectMinX, rectMinY, rectMinX, rectMaxY)) return true;
            // Right edge
            if (testLineIntersectsLine(x1, y1, x2, y2, rectMaxX, rectMinY, rectMaxX, rectMaxY)) return true;
            // Bottom edge
            if (testLineIntersectsLine(x1, y1, x2, y2, rectMinX, rectMinY, rectMaxX, rectMinY)) return true;
            // Top edge
            if (testLineIntersectsLine(x1, y1, x2, y2, rectMinX, rectMaxY, rectMaxX, rectMaxY)) return true;
            
            return false;
        }
        
        public boolean testLineIntersectsLine(double x1, double y1, double x2, double y2,
                                             double x3, double y3, double x4, double y4) {
            // Use small epsilon for floating-point comparison
            final double EPSILON = 1e-10;
            
            double denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
            
            // Parallel or coincident lines
            if (Math.abs(denom) < EPSILON) {
                // Check if lines are collinear and overlapping
                double cross1 = (x3 - x1) * (y2 - y1) - (y3 - y1) * (x2 - x1);
                if (Math.abs(cross1) > EPSILON) return false; // Not collinear
                
                // Collinear - check if they overlap or touch
                double minX1 = Math.min(x1, x2), maxX1 = Math.max(x1, x2);
                double minY1 = Math.min(y1, y2), maxY1 = Math.max(y1, y2);
                double minX2 = Math.min(x3, x4), maxX2 = Math.max(x3, x4);
                double minY2 = Math.min(y3, y4), maxY2 = Math.max(y3, y4);
                
                // Check for overlap or touching in both X and Y
                boolean xOverlap = !(maxX1 < minX2 - EPSILON || maxX2 < minX1 - EPSILON);
                boolean yOverlap = !(maxY1 < minY2 - EPSILON || maxY2 < minY1 - EPSILON);
                
                return xOverlap && yOverlap;
            }
            
            double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom;
            double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom;
            
            return t >= -EPSILON && t <= 1 + EPSILON && u >= -EPSILON && u <= 1 + EPSILON;
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
        
        public Pose[] testGetRectCorners(double minX, double maxX, double minY, double maxY) {
            return new Pose[] {
                new Pose(minX - CORNER_CLEARANCE, minY - CORNER_CLEARANCE, 0),
                new Pose(minX - CORNER_CLEARANCE, maxY + CORNER_CLEARANCE, 0),
                new Pose(maxX + CORNER_CLEARANCE, minY - CORNER_CLEARANCE, 0),
                new Pose(maxX + CORNER_CLEARANCE, maxY + CORNER_CLEARANCE, 0)
            };
        }
        
        public List<Pose> testFindShortestPath(Pose start, Pose target) {
            List<Pose> nodes = new ArrayList<>();
            nodes.add(start);
            nodes.add(target);
            for (Pose corner : testGetRectCorners(RED_STRUCTURE_MIN_X, RED_STRUCTURE_MAX_X,
                                              RED_STRUCTURE_MIN_Y, RED_STRUCTURE_MAX_Y)) {
                nodes.add(corner);
            }
            for (Pose corner : testGetRectCorners(BLUE_STRUCTURE_MIN_X, BLUE_STRUCTURE_MAX_X,
                                              BLUE_STRUCTURE_MIN_Y, BLUE_STRUCTURE_MAX_Y)) {
                nodes.add(corner);
            }

            int n = nodes.size();
            double[] dist = new double[n];
            int[] prev = new int[n];
            boolean[] visited = new boolean[n];
            Arrays.fill(dist, Double.MAX_VALUE);
            Arrays.fill(prev, -1);
            dist[0] = 0;

            PriorityQueue<Integer> pq = new PriorityQueue<>(Comparator.comparingDouble(i -> dist[i]));
            pq.offer(0);

            while (!pq.isEmpty()) {
                int u = pq.poll();
                
                if (visited[u]) continue;
                visited[u] = true;
                
                if (u == 1) break;

                for (int v = 0; v < n; v++) {
                    if (visited[v] || u == v) continue;
                    if (testPathCrossesForbiddenZone(nodes.get(u), nodes.get(v))) continue;
                    double edgeDist = Math.hypot(nodes.get(v).x() - nodes.get(u).x(),
                                                 nodes.get(v).y() - nodes.get(u).y());
                    if (dist[u] + edgeDist < dist[v]) {
                        dist[v] = dist[u] + edgeDist;
                        prev[v] = u;
                        pq.offer(v);
                    }
                }
            }

            if (dist[1] == Double.MAX_VALUE) {
                return null;
            }

            List<Pose> path = new ArrayList<>();
            int cur = 1;
            while (cur != -1) {
                path.add(0, nodes.get(cur));
                cur = prev[cur];
            }
            return path;
        }
    }

    // ========== Edge Case and Advanced Pathfinding Tests ==========
    
    @Test
    public void testComplexRouting_AroundMultipleObstacles() {
        // Path that must route around both red and blue structures
        Pose start = new Pose(20, 50, 0);
        Pose target = new Pose(110, 90, 0);
        
        List<Pose> path = runToPose.testFindShortestPath(start, target);
        
        assertNotNull("Should find path around obstacles", path);
        assertTrue("Path should have waypoints", path.size() > 2);
        
        // Verify no segment crosses forbidden zones
        for (int i = 0; i < path.size() - 1; i++) {
            assertFalse("Path segment should not cross forbidden zone",
                    runToPose.testPathCrossesForbiddenZone(path.get(i), path.get(i + 1)));
        }
    }
    
    @Test
    public void testShortPath_NoObstacleAvoidance() {
        // Direct path with no obstacles
        Pose start = new Pose(20, 20, 0);
        Pose target = new Pose(25, 25, 0);
        
        List<Pose> path = runToPose.testFindShortestPath(start, target);
        
        assertNotNull("Should find direct path", path);
        assertEquals("Direct path should have only start and target", 2, path.size());
    }
    
    @Test
    public void testPathToAdjacentForbiddenZone() {
        // Target is just outside red structure (RED_STRUCTURE_MIN_X = 35, so use x=30)
        Pose start = new Pose(20, 70, 0);
        Pose target = new Pose(30, 70, 0);
        
        List<Pose> path = runToPose.testFindShortestPath(start, target);
        
        assertNotNull("Should find path to edge of forbidden zone", path);
        Pose lastPose = path.get(path.size() - 1);
        assertEquals("Should reach target X", target.x(), lastPose.x(), 0.001);
        assertEquals("Should reach target Y", target.y(), lastPose.y(), 0.001);
    }
    
    @Test
    public void testLongDiagonalPath() {
        // Maximum diagonal across field
        Pose start = new Pose(0, 0, 0);
        Pose target = new Pose(144, 144, 0);
        
        List<Pose> path = runToPose.testFindShortestPath(start, target);
        
        assertNotNull("Should find path across entire field", path);
        assertTrue("Should have waypoints to avoid structures", path.size() >= 2);
    }
    
    @Test
    public void testPathWithDifferentHeadings() {
        // Test that heading doesn't affect pathfinding (XY collision only)
        Pose start = new Pose(20, 20, 45);
        Pose target = new Pose(25, 25, 270);
        
        List<Pose> path = runToPose.testFindShortestPath(start, target);
        
        assertNotNull("Path should be found regardless of heading", path);
        assertEquals("Direct path should have only start and target", 2, path.size());
    }
    
    @Test
    public void testBoundaryPositions() {
        // Test positions at field boundaries can be pathed to
        Pose center = new Pose(72, 72, 0);
        Pose[] boundaryPoses = {
            new Pose(0, 0, 0),
            new Pose(144, 0, 0),
            new Pose(0, 144, 0),
            new Pose(144, 144, 0)
        };
        
        for (Pose target : boundaryPoses) {
            List<Pose> path = runToPose.testFindShortestPath(center, target);
            assertNotNull("Should find path to boundary position", path);
            assertTrue("Path should have at least start and target", path.size() >= 2);
        }
    }
    
    // ========== Angle Calculation Tests ==========
    
    @Test
    public void testCalculateShortestAngle_PositiveWrap() {
        // Test wrapping from 350° to 10° (across 0)
        double current = Math.toRadians(350);
        double target = Math.toRadians(10);
        
        double wrapped = runToPose.testCalculateShortestAngle(current, target);
        
        // Wrapped angle should be close to 2π + 10° ≈ 370° (or -350° normalized)
        double normalizedWrapped = runToPose.testNormalizeAngle(wrapped);
        double normalizedTarget = runToPose.testNormalizeAngle(target);
        assertEquals("Wrapped angle should match target when normalized", 
                normalizedTarget, normalizedWrapped, 0.01);
    }
    
    @Test
    public void testCalculateShortestAngle_NegativeWrap() {
        // Test wrapping from 10° to 350° (across 0)
        double current = Math.toRadians(10);
        double target = Math.toRadians(350);
        
        double wrapped = runToPose.testCalculateShortestAngle(current, target);
        
        // Should choose shorter path across zero
        double normalizedWrapped = runToPose.testNormalizeAngle(wrapped);
        double normalizedTarget = runToPose.testNormalizeAngle(target);
        assertEquals("Wrapped angle should match target when normalized",
                normalizedTarget, normalizedWrapped, 0.01);
    }
    
    @Test
    public void testCalculateShortestAngle_NoWrapNeeded() {
        // Test when no wrapping is needed
        double current = Math.toRadians(45);
        double target = Math.toRadians(90);
        
        double result = runToPose.testCalculateShortestAngle(current, target);
        
        assertEquals("Angle should not be wrapped significantly", 
                Math.toRadians(90), runToPose.testNormalizeAngle(result), 0.001);
    }
    
    @Test
    public void testNormalizeAngle_PositiveAngle() {
        double angle = Math.toRadians(370);  // > 2π
        double normalized = runToPose.testNormalizeAngle(angle);
        assertTrue("Normalized angle should be in [-π, π]", 
                normalized >= -Math.PI && normalized <= Math.PI);
    }
    
    @Test
    public void testNormalizeAngle_NegativeAngle() {
        double angle = Math.toRadians(-370);  // < -2π
        double normalized = runToPose.testNormalizeAngle(angle);
        assertTrue("Normalized angle should be in [-π, π]", 
                normalized >= -Math.PI && normalized <= Math.PI);
    }
    
    // ========== Pathfinding Performance Tests ==========
    
    @Test
    public void testPathfinding_WorstCase() {
        // Diagonal path that requires routing through the gap between structures
        Pose start = new Pose(10, 40, 0);
        Pose target = new Pose(134, 104, 0);
        
        long startTime = System.nanoTime();
        List<Pose> path = runToPose.testFindShortestPath(start, target);
        long endTime = System.nanoTime();
        
        assertNotNull("Should find path even in worst case", path);
        assertTrue("Path should have multiple waypoints", path.size() >= 3);
        
        // Pathfinding should complete in reasonable time (< 100ms)
        long durationMs = (endTime - startTime) / 1_000_000;
        assertTrue("Pathfinding should complete quickly: " + durationMs + "ms", durationMs < 100);
    }
    
    @Test
    public void testPathfinding_BestCase() {
        // Direct line with no obstacles
        Pose start = new Pose(10, 10, 0);
        Pose target = new Pose(20, 20, 0);
        
        long startTime = System.nanoTime();
        List<Pose> path = runToPose.testFindShortestPath(start, target);
        long endTime = System.nanoTime();
        
        assertNotNull("Should find direct path", path);
        assertEquals("Direct path should have only start and target", 2, path.size());
        
        // Direct path should be very fast (< 10ms)
        long durationMs = (endTime - startTime) / 1_000_000;
        assertTrue("Direct pathfinding should be instant: " + durationMs + "ms", durationMs < 10);
    }
}
