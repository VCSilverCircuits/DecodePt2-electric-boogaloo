package org.firstinspires.ftc.teamcode.Subsystems.FlywheelConstants;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class AutoFlywheelConstants {

    private DcMotorEx leftFlywheel, rightFlywheel;
    private Servo hoodServo;
    private PIDFController VelocityPIDF;

    public static final double TICKS_PER_REV = 28;

    // ================= LINEAR REGRESSION CONSTANTS =================

    // RPM Regression
    private static final double RPM_SLOPE = 21.35148;
    private static final double RPM_INTERCEPT = 2434.33841;

    // Hood Regression
    private static final double HOOD_SLOPE = -0.880766;
    private static final double HOOD_INTERCEPT = 139.12875;

    private final double velocityCompGain = 2.0;

    // Alliance-Based Target Positions
    private final double goalX;
    private final double goalY;

    private Follower follower;

    private double targetRPM = 0;
    private double targetHoodAngle = 0;
    private boolean enabled = false;

    /**
     * @param hardwareMap FTC hardware map
     * @param follower    Odometry follower
     * @param isRed       TRUE = Red alliance, FALSE = Blue alliance
     */
    public AutoFlywheelConstants(HardwareMap hardwareMap, Follower follower, boolean isRed) {

        this.follower = follower;

        leftFlywheel = hardwareMap.get(DcMotorEx.class, "Output1");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "Output2");
        hoodServo = hardwareMap.get(Servo.class, "Hood");

        leftFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFlywheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFlywheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        VelocityPIDF = new PIDFController(0.0085, 0.007, 0, 0.0);

        leftFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // ================= ALLIANCE TARGET =================
        if (isRed) {
            goalX = 152.0;
            goalY = 142.0;
        } else {
            goalX = 0;
            goalY = 142.0;
        }
    }

    public void update(double robotForwardVelocity) {

        if (!enabled) {
            stop();
            return;
        }

        double robotX = follower.getPose().getX();
        double robotY = follower.getPose().getY();

        double dx = goalX - robotX;
        double dy = goalY - robotY;
        double distance = Math.hypot(dx, dy);

        double baseRPM = getRegressionRPM(distance);
        double baseHood = getRegressionHood(distance);

        targetRPM = baseRPM + velocityCompGain * robotForwardVelocity;
        targetHoodAngle = baseHood;

        applyRPM();
        applyHoodAngle(targetHoodAngle);
    }

    // ================= RPM REGRESSION =================
    private double getRegressionRPM(double distance) {
        double rpm = RPM_SLOPE * distance + RPM_INTERCEPT;
        return Math.max(1800, Math.min(6000, rpm));
    }

    // ================= HOOD REGRESSION =================
    private double getRegressionHood(double distance) {
        double angle = HOOD_SLOPE * distance + HOOD_INTERCEPT;
        return Math.max(0, Math.min(120, angle));
    }

    // ================= MOTOR CONTROL =================
    private void applyRPM() {
        double power = VelocityPIDF.calculate(getCurrentRPM(), targetRPM);

        power = Math.max(-1, Math.min(1, power));

        leftFlywheel.setPower((power)*1.4);
        rightFlywheel.setPower((power)*1.4);
    }

    private void applyHoodAngle(double angleDegrees) {
        double minAngle = 0;
        double maxAngle = 120;

        double position = (angleDegrees - minAngle) / (maxAngle - minAngle);
        position = Math.max(0, Math.min(1, position));

        hoodServo.setPosition((position) * 1.2);
    }

    private void stop() {
        leftFlywheel.setPower(0);
        rightFlywheel.setPower(0);
    }

    // ================= GETTERS =================
    public void enable() {
        enabled = true;
    }

    public void disable() {
        enabled = false;
        stop();
    }

    public boolean isEnabled() {
        return enabled;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getCurrentRPM() {
        double ticksPerSecond = rightFlywheel.getVelocity();
        return (ticksPerSecond * 60.0) / TICKS_PER_REV;
    }

    public double getTargetHoodAngle() {
        return targetHoodAngle;
    }

    public boolean atSpeed(double tolerance) {
        return Math.abs(getCurrentRPM() - targetRPM) < tolerance;
    }
}
