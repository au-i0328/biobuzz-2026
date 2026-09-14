package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.drivetrain.DrivePowers;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.robot.Robot;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.CRServo;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.api.PoseFactory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Configurable
public class RobotHardware {

    // Hive positions
    public static final Pose RED_START = new Pose (0, 0, 0);
    public static final Pose BLUE_START = new Pose (0, 0, 0);
    public static final Pose RED_HIVE_LEFT = new Pose(0, 0, 0);
    public static final Pose RED_HIVE_RIGHT = new Pose(57.6, 54.2, 0);
    public static final Pose BLUE_HIVE_LEFT = new Pose(0, 0, 0);
    public static final Pose BLUE_HIVE_RIGHT = new Pose(0, 0, 0);

    // Flywheel PIDF coefficients
    public static double FLYWHEEL_KP = 0.05;
    public static double FLYWHEEL_KI = 0.01;
    public static double FLYWHEEL_KD = 0.31;
    public static double FLYWHEEL_KS = 0.92;  // static friction compensation
    public static double FLYWHEEL_KV = 0.47;  // velocity feedforward
    public static double FLYWHEEL_KA = 0.3;   // acceleration feedforward
    
    // Flywheel target velocity (ticks per second)
    public static double FLYWHEEL_VELOCITY = 2000.0;
    
    // Flywheel velocity tolerance for firing
    public static double FLYWHEEL_VELOCITY_TOLERANCE = 20.0;
    
    // Alignment tolerance (degrees)
    public static double ALIGNMENT_TOLERANCE = 2.0;
    
    // Intake and transfer motor powers (volts)
    public static double INTAKE_ON_POWER = 9.0;
    public static double TRANSFER_ON_POWER = -9.0;
    public static double INTAKE_REVERSE_POWER = -9.0;
    public static double TRANSFER_REVERSE_POWER = -9.0;
    public static double TRANSFER_FIRE_POWER = 9.0;
    
    // Hardware
    public DcMotorEx frontLeft, frontRight, backLeft, backRight;
    public DcMotor intakeMotor, transferMotor;
    public MotorEx flywheelL, flywheelR;
    public CRServo intakeLeft, intakeRight;
    
    // Pedro Pathing follower (includes drivetrain and localizer)
    public Follower follower;
    
    private HardwareMap hardwareMap;
    private VoltageSensor voltageSensor;

    public void init(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        
        // Initialize voltage sensor
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        
        // Initialize Pedro Pathing follower (handles drivetrain + localizer)
        follower = Constants.create(hardwareMap);
        
        // Initialize drivetrain motors
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        
        // Configure drivetrain motors
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Initialize intake and transfer motors
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        transferMotor = hardwareMap.get(DcMotor.class, "transferMotor");
        
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        transferMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeLeft = new CRServo(hardwareMap, "intakeLeft");
        intakeRight = new CRServo(hardwareMap, "intakeRight");

        intakeRight.setInverted(true);

        // Initialize flywheel motors (Solvers Ex) with velocity control
        flywheelL = new MotorEx(hardwareMap, "flywheelL");
        flywheelR = new MotorEx(hardwareMap, "flywheelR");

        // Configure flywheel PIDF for velocity control
        flywheelL.setRunMode(MotorEx.RunMode.VelocityControl);
        flywheelL.setVeloCoefficients(FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD);
        flywheelL.setFeedforwardCoefficients(FLYWHEEL_KS, FLYWHEEL_KV, FLYWHEEL_KA);
        
        flywheelR.setRunMode(MotorEx.RunMode.VelocityControl);
        flywheelR.setVeloCoefficients(FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD);
        flywheelR.setFeedforwardCoefficients(FLYWHEEL_KS, FLYWHEEL_KV, FLYWHEEL_KA);
    }
    
    // Voltage compensation helper
    private double voltageToPower(double targetVolts) {
        double currentVoltage = voltageSensor.getVoltage();
        return targetVolts / currentVoltage;
    }
    
    // Intake control commands
    public void intakeOn() {
        intakeMotor.setPower(voltageToPower(INTAKE_ON_POWER));
        transferMotor.setPower(voltageToPower(TRANSFER_ON_POWER));
        intakeLeft.set(1);
        intakeRight.set(1);
    }
    
    public void intakeIdle() {
        intakeMotor.setPower(0);
        transferMotor.setPower(0);
        intakeLeft.set(0);
        intakeRight.set(0);
    }
    
    public void intakeReverse() {
        intakeMotor.setPower(voltageToPower(INTAKE_REVERSE_POWER));
        transferMotor.setPower(voltageToPower(TRANSFER_REVERSE_POWER));
        intakeLeft.set(-1);
        intakeRight.set(-1);
    }
    
    public void transferOn() {
        intakeMotor.setPower(voltageToPower(INTAKE_ON_POWER));
        transferMotor.setPower(voltageToPower(TRANSFER_FIRE_POWER));
        intakeLeft.set(1);
        intakeRight.set(1);
    }
    
    // Flywheel control methods
    public void setFlywheelVelocity(double velocity) {
        flywheelL.setVelocity(velocity);
        flywheelR.setVelocity(velocity);
    }
    
    public void flywheelOn() {
        setFlywheelVelocity(FLYWHEEL_VELOCITY);
    }
    
    public void flywheelOff() {
        flywheelL.stopMotor();
        flywheelR.stopMotor();
    }
    
    public double getFlywheelVelocity() {
        return (flywheelL.getVelocity() + flywheelR.getVelocity()) / 2.0;
    }
    
    // Check if flywheel is at target velocity
    public boolean isFlywheelReady() {
        double currentVel = getFlywheelVelocity();
        return Math.abs(currentVel - FLYWHEEL_VELOCITY) < FLYWHEEL_VELOCITY_TOLERANCE;
    }
}
