package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.pedropathing.follower.ManualDrive;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.curve;

public class RunToPose {
    private Follower follower;
    private OpMode opMode;
    private static final double JOYSTICK_DEADZONE = 0.1;
    
    // Hard stop zones - structure obstacles with safety margin
    private static final double SAFETY_MARGIN = 10.0; // inches buffer around obstacles
    
    private static final double RED_STRUCTURE_MIN_X = 45 - SAFETY_MARGIN;
    private static final double RED_STRUCTURE_MAX_X = 50 + SAFETY_MARGIN;
    private static final double RED_STRUCTURE_MIN_Y = 48 - SAFETY_MARGIN;
    private static final double RED_STRUCTURE_MAX_Y = 93 + SAFETY_MARGIN;
    
    private static final double BLUE_STRUCTURE_MIN_X = 92 - SAFETY_MARGIN;
    private static final double BLUE_STRUCTURE_MAX_X = 97 + SAFETY_MARGIN;
    private static final double BLUE_STRUCTURE_MIN_Y = 48 - SAFETY_MARGIN;
    private static final double BLUE_STRUCTURE_MAX_Y = 93 + SAFETY_MARGIN;


    /**
     * Constructs a RunToPose controller
     * @param follower The Pedro Pathing follower from RobotHardware
     * @param opMode The OpMode for status updates and control flow
     */
    public RunToPose(Follower follower, OpMode opMode) {
        this.follower = follower;
        this.opMode = opMode;
    }
    
    /**
     * Checks if any gamepad joystick is being moved beyond the deadzone
     * @return true if any joystick input detected
     */
    private boolean hasJoystickInput() {
        return Math.abs(opMode.gamepad1.left_stick_x) > JOYSTICK_DEADZONE ||
               Math.abs(opMode.gamepad1.left_stick_y) > JOYSTICK_DEADZONE ||
               Math.abs(opMode.gamepad1.right_stick_x) > JOYSTICK_DEADZONE ||
               Math.abs(opMode.gamepad1.right_stick_y) > JOYSTICK_DEADZONE;
    }
    
    /**
     * Checks if the OpMode should continue running
     * Works for both OpMode (loop-based) and LinearOpMode (blocking)
     */
    private boolean shouldContinue() {
        // For OpMode, we just return true and let the loop() method handle stopping
        // The OpMode framework will stop calling loop() when the OpMode is stopped
        return true;
    }
    
    /**
     * Normalizes an angle to the range [-PI, PI]
     * Fixes angle wrap-around issues
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
    
    /**
     * Calculates the shortest angular distance from current heading to target heading
     * This accounts for wrap-around and returns the angle that may be outside [-PI, PI]
     * to ensure the interpolator takes the shortest path
     */
    private double calculateShortestAngle(double currentHeading, double targetHeading) {
        // Normalize both angles first
        double current = normalizeAngle(currentHeading);
        double target = normalizeAngle(targetHeading);
        
        // Calculate the direct difference
        double diff = target - current;
        
        // If the difference is more than PI, going the other way is shorter
        if (diff > Math.PI) {
            // Going counter-clockwise (negative direction) is shorter
            return target - 2 * Math.PI;
        } else if (diff < -Math.PI) {
            // Going clockwise (positive direction) is shorter
            return target + 2 * Math.PI;
        } else {
            // Direct path is already shortest
            return target;
        }
    }
    
    /**
     * Checks if a point is inside the red structure zone
     */
    private boolean isInRedStructure(double x, double y) {
        return x >= RED_STRUCTURE_MIN_X && x <= RED_STRUCTURE_MAX_X &&
               y >= RED_STRUCTURE_MIN_Y && y <= RED_STRUCTURE_MAX_Y;
    }
    
    /**
     * Checks if a point is inside the blue structure zone
     */
    private boolean isInBlueStructure(double x, double y) {
        return x >= BLUE_STRUCTURE_MIN_X && x <= BLUE_STRUCTURE_MAX_X &&
               y >= BLUE_STRUCTURE_MIN_Y && y <= BLUE_STRUCTURE_MAX_Y;
    }
    
    /**
     * Checks if a point is inside any forbidden zone
     */
    private boolean isInForbiddenZone(double x, double y) {
        return isInRedStructure(x, y) || isInBlueStructure(x, y);
    }
    
    /**
     * Checks if a line segment intersects with a rectangle
     */
    private boolean lineIntersectsRect(double x1, double y1, double x2, double y2,
                                       double rectMinX, double rectMaxX, 
                                       double rectMinY, double rectMaxY) {
        // Check if either endpoint is inside the rectangle
        if ((x1 >= rectMinX && x1 <= rectMaxX && y1 >= rectMinY && y1 <= rectMaxY) ||
            (x2 >= rectMinX && x2 <= rectMaxX && y2 >= rectMinY && y2 <= rectMaxY)) {
            return true;
        }
        
        // Check if line crosses any of the four edges
        // Left edge
        if (lineIntersectsLine(x1, y1, x2, y2, rectMinX, rectMinY, rectMinX, rectMaxY)) return true;
        // Right edge
        if (lineIntersectsLine(x1, y1, x2, y2, rectMaxX, rectMinY, rectMaxX, rectMaxY)) return true;
        // Bottom edge
        if (lineIntersectsLine(x1, y1, x2, y2, rectMinX, rectMinY, rectMaxX, rectMinY)) return true;
        // Top edge
        if (lineIntersectsLine(x1, y1, x2, y2, rectMinX, rectMaxY, rectMaxX, rectMaxY)) return true;
        
        return false;
    }
    
    /**
     * Checks if two line segments intersect
     */
    private boolean lineIntersectsLine(double x1, double y1, double x2, double y2,
                                       double x3, double y3, double x4, double y4) {
        double denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        if (Math.abs(denom) < 1e-10) return false; // Parallel lines
        
        double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom;
        double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom;
        
        return t >= 0 && t <= 1 && u >= 0 && u <= 1;
    }
    
    /**
     * Checks if the path to target would cross any forbidden zones
     */
    private boolean pathCrossesForbiddenZone(Pose current, Pose target) {
        return lineIntersectsRect(current.x(), current.y(), target.x(), target.y(),
                                 RED_STRUCTURE_MIN_X, RED_STRUCTURE_MAX_X,
                                 RED_STRUCTURE_MIN_Y, RED_STRUCTURE_MAX_Y) ||
               lineIntersectsRect(current.x(), current.y(), target.x(), target.y(),
                                 BLUE_STRUCTURE_MIN_X, BLUE_STRUCTURE_MAX_X,
                                 BLUE_STRUCTURE_MIN_Y, BLUE_STRUCTURE_MAX_Y);
    }
    
    /**
     * Calculates a waypoint to route around obstacles
     * Returns the best waypoint that avoids forbidden zones
     */
    private Pose calculateWaypoint(Pose current, Pose target) {
        // Determine which side of obstacles to route around
        double currentX = current.x();
        double currentY = current.y();
        double targetX = target.x();
        double targetY = target.y();
        
        // Check which structure we're likely interfering with
        boolean redInWay = lineIntersectsRect(currentX, currentY, targetX, targetY,
                                              RED_STRUCTURE_MIN_X, RED_STRUCTURE_MAX_X,
                                              RED_STRUCTURE_MIN_Y, RED_STRUCTURE_MIN_Y);
        boolean blueInWay = lineIntersectsRect(currentX, currentY, targetX, targetY,
                                               BLUE_STRUCTURE_MIN_X, BLUE_STRUCTURE_MAX_X,
                                               BLUE_STRUCTURE_MIN_Y, BLUE_STRUCTURE_MAX_Y);
        
        // Calculate potential waypoints around structures
        // Route below red structure (Y < 48)
        Pose waypointBelowRed = new Pose(47.5, 45, current.heading());
        // Route above red structure (Y > 93)  
        Pose waypointAboveRed = new Pose(47.5, 96, current.heading());
        // Route below blue structure (Y < 48)
        Pose waypointBelowBlue = new Pose(94.5, 45, current.heading());
        // Route above blue structure (Y > 93)
        Pose waypointAboveBlue = new Pose(94.5, 96, current.heading());
        // Route between structures (X between 50-92, any Y)
        Pose waypointMiddle = new Pose(71, currentY, current.heading());
        
        // Find the shortest valid path
        Pose bestWaypoint = null;
        double shortestDistance = Double.MAX_VALUE;
        
        Pose[] candidates = {waypointBelowRed, waypointAboveRed, waypointBelowBlue, 
                            waypointAboveBlue, waypointMiddle};
        
        for (Pose waypoint : candidates) {
            // Check if this waypoint creates a valid path
            boolean firstLegClear = !pathCrossesForbiddenZone(current, waypoint);
            boolean secondLegClear = !pathCrossesForbiddenZone(waypoint, target);
            boolean waypointSafe = !isInForbiddenZone(waypoint.x(), waypoint.y());
            
            if (firstLegClear && secondLegClear && waypointSafe) {
                // Calculate total distance through this waypoint
                double distToWaypoint = Math.hypot(waypoint.x() - currentX, waypoint.y() - currentY);
                double distToTarget = Math.hypot(targetX - waypoint.x(), targetY - waypoint.y());
                double totalDist = distToWaypoint + distToTarget;
                
                if (totalDist < shortestDistance) {
                    shortestDistance = totalDist;
                    bestWaypoint = waypoint;
                }
            }
        }
        
        return bestWaypoint;
    }
    
    /**
     * Drives the robot to a target pose using Pedro Pathing Foresight
     * Holds position until any joystick input is detected
     * Automatically reroutes around obstacles if direct path is blocked
     * Uses smooth curve interpolation for natural motion
     * @param targetPose The pose to drive to
     */
    public void goToPose(Pose targetPose) {
        // Normalize target heading to fix angle wrap-around
        Pose normalizedTarget = new Pose(
            targetPose.x(), 
            targetPose.y(), 
            normalizeAngle(targetPose.heading())
        );
        
        Pose currentPose = follower.pose();
        
        // Check if target is in a forbidden zone
        if (isInForbiddenZone(normalizedTarget.x(), normalizedTarget.y())) {
            opMode.telemetry.addData("ERROR", "Target is inside a forbidden structure zone!");
            opMode.telemetry.addData("Target", String.format("(%.1f, %.1f)", 
                normalizedTarget.x(), normalizedTarget.y()));
            opMode.telemetry.update();
            return;
        }
        
        // Check if direct path would cross a forbidden zone - if so, reroute
        if (pathCrossesForbiddenZone(currentPose, normalizedTarget)) {
            // Calculate waypoint to route around obstacles
            Pose waypoint = calculateWaypoint(currentPose, normalizedTarget);
            
            if (waypoint == null) {
                opMode.telemetry.addData("ERROR", "Cannot find valid path around obstacles!");
                opMode.telemetry.addData("Current", String.format("(%.1f, %.1f)", 
                    currentPose.x(), currentPose.y()));
                opMode.telemetry.addData("Target", String.format("(%.1f, %.1f)", 
                    normalizedTarget.x(), normalizedTarget.y()));
                opMode.telemetry.update();
                return;
            }
            
            // Build smooth curved path with waypoint using Bezier interpolation
            opMode.telemetry.addData("Info", "Routing around obstacles with smooth curve");
            opMode.telemetry.addData("Waypoint", String.format("(%.1f, %.1f)", 
                waypoint.x(), waypoint.y()));
            opMode.telemetry.update();
            
            // Calculate control point for smooth curve
            double midX = (currentPose.x() + waypoint.x()) / 2.0;
            double midY = (currentPose.y() + waypoint.y()) / 2.0;
            Pose controlPoint1 = new Pose(midX, midY, currentPose.heading());
            
            // Calculate shortest angular path for first segment
            double waypointAngleOptimized = calculateShortestAngle(currentPose.heading(), waypoint.heading());
            
            // First segment: current -> waypoint with smooth curve
            Path path1 = curve(currentPose, controlPoint1, waypoint)
                .linear(currentPose.heading(), waypointAngleOptimized);
            follower.follow(path1);
            
            while (follower.isBusy() && shouldContinue()) {
                follower.update();
                
                Pose currentPos = follower.pose();
                if (isInForbiddenZone(currentPos.x(), currentPos.y())) {
                    follower.manual();
                    opMode.telemetry.addData("EMERGENCY STOP", "Robot entered forbidden zone!");
                    opMode.telemetry.addData("Position", String.format("(%.1f, %.1f)", 
                        currentPos.x(), currentPos.y()));
                    opMode.telemetry.update();
                    return;
                }
                
                opMode.telemetry.addData("Status", "Driving to waypoint (curved path)");
                opMode.telemetry.addData("Waypoint", String.format("(%.1f, %.1f)", 
                    waypoint.x(), waypoint.y()));
                opMode.telemetry.update();
            }
            
            // Calculate control point for second smooth curve
            Pose currentAfterWaypoint = follower.pose();
            double midX2 = (currentAfterWaypoint.x() + normalizedTarget.x()) / 2.0;
            double midY2 = (currentAfterWaypoint.y() + normalizedTarget.y()) / 2.0;
            Pose controlPoint2 = new Pose(midX2, midY2, currentAfterWaypoint.heading());
            
            // Calculate shortest angular path for second segment
            double targetAngleOptimized = calculateShortestAngle(currentAfterWaypoint.heading(), normalizedTarget.heading());
            
            // Second segment: waypoint -> target with smooth curve
            Path path2 = curve(currentAfterWaypoint, controlPoint2, normalizedTarget)
                .linear(currentAfterWaypoint.heading(), targetAngleOptimized);
            follower.follow(path2);
        } else {
            // Direct path is clear - use smooth curve for natural motion
            double midX = (currentPose.x() + normalizedTarget.x()) / 2.0;
            double midY = (currentPose.y() + normalizedTarget.y()) / 2.0;
            Pose controlPoint = new Pose(midX, midY, currentPose.heading());
            
            // Calculate shortest angular path
            double targetAngleOptimized = calculateShortestAngle(currentPose.heading(), normalizedTarget.heading());
            
            Path path = curve(currentPose, controlPoint, normalizedTarget)
                .linear(currentPose.heading(), targetAngleOptimized);
            follower.follow(path);
        }
        
        // Wait for path to complete - use isBusy() instead of atParametricEnd()
        while (follower.isBusy() && shouldContinue()) {
            follower.update();
            
            // Hard stop if robot enters a forbidden zone
            Pose currentPos = follower.pose();
            if (isInForbiddenZone(currentPos.x(), currentPos.y())) {
                follower.manual();
                opMode.telemetry.addData("EMERGENCY STOP", "Robot entered forbidden zone!");
                opMode.telemetry.addData("Position", String.format("(%.1f, %.1f)", 
                    currentPos.x(), currentPos.y()));
                opMode.telemetry.update();
                return;
            }
            
            opMode.telemetry.addData("Status", "Driving to pose");
            opMode.telemetry.addData("Target", String.format("(%.1f, %.1f, %.1f°)", 
                normalizedTarget.x(), normalizedTarget.y(), Math.toDegrees(normalizedTarget.heading())));
            opMode.telemetry.addData("Current", String.format("(%.1f, %.1f, %.1f°)", 
                currentPos.x(), currentPos.y(), Math.toDegrees(currentPos.heading())));
            opMode.telemetry.update();
        }
        
        // Hold position until joystick input
        follower.hold(normalizedTarget);
        while (!hasJoystickInput() && shouldContinue()) {
            follower.update();
            opMode.telemetry.addData("Status", "Holding pose - move joystick to take control");
            opMode.telemetry.addData("Current", String.format("(%.1f, %.1f, %.1f°)", 
                follower.pose().x(), follower.pose().y(), Math.toDegrees(follower.pose().heading())));
            opMode.telemetry.update();
        }
        
        // Set follower to manual mode when hold is stopped
        follower.manual();
    }
    
    /**
     * Drives to RED_HIVE_LEFT position
     */
    public void goToRedHiveLeft() {
        goToPose(RobotHardware.RED_HIVE_LEFT_SHOOT);
    }
    
    /**
     * Drives to RED_HIVE_RIGHT position
     */
    public void goToRedHiveRight() {
        goToPose(RobotHardware.RED_HIVE_RIGHT_SHOOT);
    }
    
    /**
     * Drives to BLUE_HIVE_LEFT position
     */
    public void goToBlueHiveLeft() {
        goToPose(RobotHardware.BLUE_HIVE_LEFT_SHOOT);
    }
    
    /**
     * Drives to BLUE_HIVE_RIGHT position
     */
    public void goToBlueHiveRight() {
        goToPose(RobotHardware.BLUE_HIVE_RIGHT_SHOOT);
    }
    
    /**
     * Drives to RED_START position
     */
    public void goToRedStart() {
        goToPose(RobotHardware.RED_START);
    }
    
    /**
     * Drives to BLUE_START position
     */
    public void goToBlueStart() {
        goToPose(RobotHardware.BLUE_START);
    }

    public void goToZero() {
        goToPose(new Pose(72,72,0));
    }

    /**
     * Checks if the robot is currently executing a path
     * @return true if busy following a path, false otherwise
     */
    public boolean isBusy() {
        return follower.isBusy();
    }
    
    /**
     * Updates the follower - call this in your OpMode loop if you want async path following
     */
    public void update() {
        follower.update();
    }
}
