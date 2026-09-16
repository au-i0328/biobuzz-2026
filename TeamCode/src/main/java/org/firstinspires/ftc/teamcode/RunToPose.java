package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.curve;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.PriorityQueue;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;

public class RunToPose {
    private Follower follower;
    private OpMode opMode;
    private static final double JOYSTICK_DEADZONE = 0.1;
    
    // Async pathfinding state
    private CompletableFuture<List<Pose>> pendingPathfinding = null;
    private List<Pose> cachedRoute = null;
    
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
        // Check if either endpoint is inside the rectangle (including boundary)
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
    
    // How far outside a rectangle's corner to place a visibility-graph node.
    // Keeps nodes from sitting exactly on the boundary (which would count as "inside").
    private static final double CORNER_CLEARANCE = 1.0;

    /**
     * Returns the 4 corners of a rectangle, each nudged outward (away from the
     * rectangle) by CORNER_CLEARANCE so the corner itself is not inside the zone.
     */
    private Pose[] getRectCorners(double minX, double maxX, double minY, double maxY) {
        return new Pose[] {
            new Pose(minX - CORNER_CLEARANCE, minY - CORNER_CLEARANCE, 0),
            new Pose(minX - CORNER_CLEARANCE, maxY + CORNER_CLEARANCE, 0),
            new Pose(maxX + CORNER_CLEARANCE, minY - CORNER_CLEARANCE, 0),
            new Pose(maxX + CORNER_CLEARANCE, maxY + CORNER_CLEARANCE, 0)
        };
    }

    /**
     * Finds the shortest sequence of straight-line hops from start to target that
     * never cuts through either forbidden zone, using a visibility graph over the
     * rectangle corners plus Dijkstra's algorithm with priority queue (O(n log n)).
     * Returns the full waypoint list including start (index 0) and target (last index),
     * or null if no path exists.
     */
    private List<Pose> findShortestPath(Pose start, Pose target) {
        List<Pose> nodes = new ArrayList<>();
        nodes.add(start);
        nodes.add(target);
        for (Pose corner : getRectCorners(RED_STRUCTURE_MIN_X, RED_STRUCTURE_MAX_X,
                                          RED_STRUCTURE_MIN_Y, RED_STRUCTURE_MAX_Y)) {
            nodes.add(corner);
        }
        for (Pose corner : getRectCorners(BLUE_STRUCTURE_MIN_X, BLUE_STRUCTURE_MAX_X,
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

        // Use priority queue for O(n log n) instead of O(n²)
        PriorityQueue<Integer> pq = new PriorityQueue<>(Comparator.comparingDouble(i -> dist[i]));
        pq.offer(0);

        while (!pq.isEmpty()) {
            int u = pq.poll();
            
            if (visited[u]) continue;
            visited[u] = true;
            
            if (u == 1) break; // Found target

            for (int v = 0; v < n; v++) {
                if (visited[v] || u == v) continue;
                if (pathCrossesForbiddenZone(nodes.get(u), nodes.get(v))) continue;
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
            return null; // no valid route found
        }

        // Walk back from target (index 1) to start (index 0)
        List<Pose> path = new ArrayList<>();
        int cur = 1;
        while (cur != -1) {
            path.add(0, nodes.get(cur));
            cur = prev[cur];
        }
        return path;
    }
    
    /**
     * Starts async pathfinding computation in background thread.
     * Non-blocking - returns immediately.
     */
    private void startAsyncPathfinding(Pose start, Pose target) {
        pendingPathfinding = CompletableFuture.supplyAsync(() -> findShortestPath(start, target));
    }
    
    /**
     * Checks if async pathfinding is complete and retrieves result.
     * Returns null if not yet complete or if pathfinding failed.
     */
    private List<Pose> getAsyncPathfindingResult() {
        if (pendingPathfinding == null) return null;
        if (!pendingPathfinding.isDone()) return null;
        
        try {
            List<Pose> result = pendingPathfinding.get();
            pendingPathfinding = null;
            return result;
        } catch (InterruptedException | ExecutionException e) {
            opMode.telemetry.addData("Pathfinding Error", e.getMessage());
            pendingPathfinding = null;
            return null;
        }
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
            // Start async pathfinding if not already running
            if (pendingPathfinding == null && cachedRoute == null) {
                startAsyncPathfinding(currentPose, normalizedTarget);
                opMode.telemetry.addData("Info", "Computing path around obstacles...");
                opMode.telemetry.update();
                follower.manual();
                return; // Exit and wait for next call
            }
            
            // Check if pathfinding completed
            if (cachedRoute == null) {
                cachedRoute = getAsyncPathfindingResult();
                if (cachedRoute == null) {
                    // Still computing
                    opMode.telemetry.addData("Info", "Computing path around obstacles...");
                    opMode.telemetry.update();
                    follower.manual();
                    return;
                }
            }
            
            // Validate computed route
            if (cachedRoute.size() < 2) {
                opMode.telemetry.addData("ERROR", "Cannot find valid path around obstacles!");
                opMode.telemetry.addData("Current", String.format("(%.1f, %.1f)",
                    currentPose.x(), currentPose.y()));
                opMode.telemetry.addData("Target", String.format("(%.1f, %.1f)",
                    normalizedTarget.x(), normalizedTarget.y()));
                opMode.telemetry.update();
                cachedRoute = null;
                follower.manual();
                return;
            }

            opMode.telemetry.addData("Info", "Following computed path around obstacles");
            opMode.telemetry.update();

            // Follow each leg of the shortest route in turn
            for (int i = 1; i < cachedRoute.size(); i++) {
                Pose legStart = follower.pose();
                Pose legEnd = cachedRoute.get(i);
                boolean isFinalLeg = (i == cachedRoute.size() - 1);
                Pose legEndWithHeading = isFinalLeg
                    ? normalizedTarget
                    : new Pose(legEnd.x(), legEnd.y(), legStart.heading());

                double midX = (legStart.x() + legEndWithHeading.x()) / 2.0;
                double midY = (legStart.y() + legEndWithHeading.y()) / 2.0;
                Pose controlPoint = new Pose(midX, midY, legStart.heading());

                double angleOptimized = calculateShortestAngle(legStart.heading(), legEndWithHeading.heading());

                Path leg = curve(legStart, controlPoint, legEndWithHeading)
                    .linear(legStart.heading(), angleOptimized);
                follower.follow(leg);

                while (follower.isBusy() && shouldContinue()) {
                    follower.update();

                    opMode.telemetry.addData("Status", "Driving around obstacles (leg " + i + "/" + (cachedRoute.size() - 1) + ")");
                    opMode.telemetry.addData("Next waypoint", String.format("(%.1f, %.1f)",
                        legEndWithHeading.x(), legEndWithHeading.y()));
                    opMode.telemetry.update();
                }
            }
            
            // Clear cached route after use
            cachedRoute = null;
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
            
            opMode.telemetry.addData("Status", "Driving to pose");
            opMode.telemetry.addData("Target", String.format("(%.1f, %.1f, %.1f°)", 
                normalizedTarget.x(), normalizedTarget.y(), Math.toDegrees(normalizedTarget.heading())));
            opMode.telemetry.addData("Current", String.format("(%.1f, %.1f, %.1f°)", 
                follower.pose().x(), follower.pose().y(), Math.toDegrees(follower.pose().heading())));
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
