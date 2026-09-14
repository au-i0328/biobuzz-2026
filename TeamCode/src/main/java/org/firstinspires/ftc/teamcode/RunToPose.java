package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import static com.pedropathing.api.Paths.line;

public class RunToPose {
    private Follower follower;
    private OpMode opMode;
    private static final double JOYSTICK_DEADZONE = 0.1;
    
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
     * Drives the robot to a target pose using Pedro Pathing Foresight
     * Holds position until any joystick input is detected
     * @param targetPose The pose to drive to
     */
    public void goToPose(Pose targetPose) {
        // Build a line path from current pose to target pose with linear heading interpolation
        Pose currentPose = follower.pose();
        Path path = line(currentPose, targetPose).linear(currentPose.heading(), targetPose.heading());
        
        // Follow the path
        follower.follow(path);
        
        // Wait for path to complete
        while (!follower.atParametricEnd() && shouldContinue()) {
            follower.update();
            opMode.telemetry.addData("Status", "Driving to pose");
            opMode.telemetry.addData("Target", String.format("(%.1f, %.1f, %.1f°)", 
                targetPose.x(), targetPose.y(), Math.toDegrees(targetPose.heading())));
            opMode.telemetry.addData("Current", String.format("(%.1f, %.1f, %.1f°)", 
                follower.pose().x(), follower.pose().y(), Math.toDegrees(follower.pose().heading())));
            opMode.telemetry.update();
        }
        
        // Hold position until joystick input
        follower.hold(targetPose);
        while (!hasJoystickInput() && shouldContinue()) {
            follower.update();
            opMode.telemetry.addData("Status", "Holding pose - move joystick to take control");
            opMode.telemetry.addData("Current", String.format("(%.1f, %.1f, %.1f°)", 
                follower.pose().x(), follower.pose().y(), Math.toDegrees(follower.pose().heading())));
            opMode.telemetry.update();
        }
    }
    
    /**
     * Drives to RED_HIVE_LEFT position
     */
    public void goToRedHiveLeft() {
        goToPose(RobotHardware.RED_HIVE_LEFT);
    }
    
    /**
     * Drives to RED_HIVE_RIGHT position
     */
    public void goToRedHiveRight() {
        goToPose(RobotHardware.RED_HIVE_RIGHT);
    }
    
    /**
     * Drives to BLUE_HIVE_LEFT position
     */
    public void goToBlueHiveLeft() {
        goToPose(RobotHardware.BLUE_HIVE_LEFT);
    }
    
    /**
     * Drives to BLUE_HIVE_RIGHT position
     */
    public void goToBlueHiveRight() {
        goToPose(RobotHardware.BLUE_HIVE_RIGHT);
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
    
    /**
     * Checks if the robot is currently executing a path
     * @return true if at the end of path, false otherwise
     */
    public boolean atEnd() {
        return follower.atParametricEnd();
    }
    
    /**
     * Updates the follower - call this in your OpMode loop if you want async path following
     */
    public void update() {
        follower.update();
    }
}
