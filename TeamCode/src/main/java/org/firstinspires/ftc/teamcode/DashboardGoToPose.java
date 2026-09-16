package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Configurable
@TeleOp(name = "Panels Go To Pose", group = "Navigation")
public class DashboardGoToPose extends OpMode {
    
    private RobotHardware robot;
    private RunToPose runToPose;
    
    // Configurable target pose variables exposed to Panels Dashboard
    public static double TARGET_X = 72.0;
    public static double TARGET_Y = 72.0;
    public static double TARGET_HEADING_DEGREES = 0.0;
    
    // Control variables
    public static boolean GO_TO_POSE = false;
    public static boolean RESET_POSITION = false;
    
    // State tracking
    private boolean lastGoToPoseState = false;
    private boolean lastResetState = false;
    private boolean isDriving = false;
    
    @Override
    public void init() {
        // Initialize robot hardware
        robot = new RobotHardware();
        robot.init(hardwareMap);
        runToPose = new RunToPose(robot.follower, this);
        
        PanelsTelemetry.INSTANCE.getTelemetry().debug("Panels Go To Pose Initialized");
        PanelsTelemetry.INSTANCE.getTelemetry().debug("");
        PanelsTelemetry.INSTANCE.getTelemetry().debug("Use Panels Dashboard to configure target pose:");
        PanelsTelemetry.INSTANCE.getTelemetry().debug("  - Set TARGET_X, TARGET_Y, TARGET_HEADING_DEGREES");
        PanelsTelemetry.INSTANCE.getTelemetry().debug("  - Toggle GO_TO_POSE to true to drive");
        PanelsTelemetry.INSTANCE.getTelemetry().debug("  - Toggle RESET_POSITION to reset odometry");
        PanelsTelemetry.INSTANCE.getTelemetry().debug("");
        PanelsTelemetry.INSTANCE.getTelemetry().debug("Use gamepad joysticks to take manual control");
        PanelsTelemetry.INSTANCE.getTelemetry().update();
    }
    
    @Override
    public void init_loop() {
        PanelsTelemetry.INSTANCE.getTelemetry().debug("Status: Ready to start");
        PanelsTelemetry.INSTANCE.getTelemetry().debug("Current Pose: " + formatPose(robot.follower.pose()));
        PanelsTelemetry.INSTANCE.getTelemetry().debug(String.format("Target Pose: (%.1f, %.1f, %.1f°)", 
            TARGET_X, TARGET_Y, TARGET_HEADING_DEGREES));
        PanelsTelemetry.INSTANCE.getTelemetry().update();
    }
    
    @Override
    public void start() {
        robot.follower.manual();
        PanelsTelemetry.INSTANCE.getTelemetry().debug("OpMode Started");
        PanelsTelemetry.INSTANCE.getTelemetry().debug("Configure target pose in Panels Dashboard and toggle GO_TO_POSE");
        PanelsTelemetry.INSTANCE.getTelemetry().update();
    }
    
    @Override
    public void loop() {
        // Update follower
        robot.follower.update();
        
        // Check for position reset request (rising edge detection)
        if (RESET_POSITION && !lastResetState) {
            robot.follower.setPose(new Pose(0, 0, 0));
            PanelsTelemetry.INSTANCE.getTelemetry().debug("Action: Position reset to origin");
            RESET_POSITION = false;
        }
        lastResetState = RESET_POSITION;
        
        // Check for GO_TO_POSE toggle (rising edge detection)
        if (GO_TO_POSE && !lastGoToPoseState && !isDriving) {
            // Convert heading from degrees to radians
            double targetHeadingRad = Math.toRadians(TARGET_HEADING_DEGREES);
            Pose targetPose = new Pose(TARGET_X, TARGET_Y, targetHeadingRad);
            
            PanelsTelemetry.INSTANCE.getTelemetry().debug("Action: Driving to pose");
            PanelsTelemetry.INSTANCE.getTelemetry().debug("Target: " + formatPose(targetPose));
            PanelsTelemetry.INSTANCE.getTelemetry().update();
            
            // Execute go to pose command
            isDriving = true;
            runToPose.goToPose(targetPose);
            isDriving = false;
            
            // Reset toggle after execution
            GO_TO_POSE = false;
        }
        lastGoToPoseState = GO_TO_POSE;
        
        // Display current status
        Pose currentPose = robot.follower.pose();
        PanelsTelemetry.INSTANCE.getTelemetry().debug("Status: " + (isDriving ? "Driving to pose" : "Ready"));
        PanelsTelemetry.INSTANCE.getTelemetry().debug("Current Pose: " + formatPose(currentPose));
        PanelsTelemetry.INSTANCE.getTelemetry().debug(String.format("Target Pose: (%.1f, %.1f, %.1f°)", 
            TARGET_X, TARGET_Y, TARGET_HEADING_DEGREES));
        PanelsTelemetry.INSTANCE.getTelemetry().debug("");
        PanelsTelemetry.INSTANCE.getTelemetry().debug("GO_TO_POSE: " + (GO_TO_POSE ? "TRUE - Will execute" : "false"));
        PanelsTelemetry.INSTANCE.getTelemetry().debug("RESET_POSITION: " + (RESET_POSITION ? "TRUE - Will reset" : "false"));
        PanelsTelemetry.INSTANCE.getTelemetry().debug("");
        PanelsTelemetry.INSTANCE.getTelemetry().debug("Instructions:");
        PanelsTelemetry.INSTANCE.getTelemetry().debug("1. Open Panels Dashboard on your phone/tablet");
        PanelsTelemetry.INSTANCE.getTelemetry().debug("2. Adjust TARGET_X, TARGET_Y, TARGET_HEADING_DEGREES");
        PanelsTelemetry.INSTANCE.getTelemetry().debug("3. Toggle GO_TO_POSE to true");
        PanelsTelemetry.INSTANCE.getTelemetry().debug("4. Robot will drive to pose with obstacle avoidance");
        PanelsTelemetry.INSTANCE.getTelemetry().debug("5. Move joystick to take manual control");
        
        PanelsTelemetry.INSTANCE.getTelemetry().update();
    }
    
    @Override
    public void stop() {
        robot.follower.manual();
    }
    
    /**
     * Formats a pose for display
     */
    private String formatPose(Pose pose) {
        return String.format("(%.1f, %.1f, %.1f°)", 
            pose.x(), pose.y(), Math.toDegrees(pose.heading()));
    }
}
