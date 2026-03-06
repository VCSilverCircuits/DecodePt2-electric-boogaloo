package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Configurable
public class OdoAim {

    private DcMotorEx yawMotor;
    private Follower follower;
    private Limelight3A limelight;

    private double MAX_TURRET_RAD = Math.toRadians(175);
    private double MIN_TURRET_RAD = Math.toRadians(-90);

    private double turretPosition;
    private double isLLgetting;
    private Pose targetPose; // current target, can change via D-pad

    public static double RADIANSPERTICK = 0.001062;

    // ================= OFFSET SYSTEM =================
    private double manualOffsetRad = 0.0;
    public static double OFFSET_STEP_RAD = Math.toRadians(3.0);

     Pose REDTARGET = new Pose(152.0 , 142.0 );
     Pose BLUETARGET = new Pose(-3, 140.0);

    private final PIDFController limelightPIDF =
        new PIDFController(0.06, 0.0, 0.008, 0.0);

    private final PIDFController odometryPIDF =
        new PIDFController(2.4, 0.0, 0.008, 0.2);

    private double relativeTargetHeading;
    private boolean isRed;
    double targetX = isRed ? REDTARGET.getX() : BLUETARGET.getX();
    double targetY = isRed ? REDTARGET.getY() : BLUETARGET.getY();

    public OdoAim(HardwareMap hardwareMap, Follower follower, boolean isRed) {

        this.follower = follower;
        this.targetPose = isRed ? REDTARGET : BLUETARGET;

        yawMotor = hardwareMap.get(DcMotorEx.class, "turretRotation");
        yawMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yawMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(isRed ? 4 : 0);
        limelight.start();
    }

    // ================= UPDATE TARGET ANGLE =================
    public void update() {
        turretPosition = yawMotor.getCurrentPosition() * RADIANSPERTICK;

        double robotX = follower.getPose().getX();
        double robotY = follower.getPose().getY();
        double robotHeading = follower.getHeading();

        double fieldAngleToTarget =
            Math.atan2(targetPose.getY() - robotY, targetPose.getX() - robotX);

        relativeTargetHeading =
            AngleUnit.normalizeRadians(fieldAngleToTarget - robotHeading);
    }
    // ================= ODOMETRY AIM WITH OFFSET =================
    public void odoAim() {

        double targetAngle =
            AngleUnit.normalizeRadians(relativeTargetHeading + manualOffsetRad);

        // HARD LIMIT CLAMP
        targetAngle = Math.max(MIN_TURRET_RAD,
            Math.min(MAX_TURRET_RAD, targetAngle));

        double power = odometryPIDF.calculate(turretPosition, targetAngle);

        yawMotor.setPower(normalizePower(power));
    }

    // ================= LIMELIGHT AIM =================
    public void aim() {

        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()) {

            double power = limelightPIDF.calculate(llResult.getTx(), 0);
            isLLgetting = power;
            yawMotor.setPower(normalizePower(power));

        } else {
            odoAim();
        }
    }

    // ================= OFFSET CONTROLS =================
    public void changeTarget(boolean dpadRight, boolean dpadLeft) {
        if (dpadRight) {
            // Example: move turret target slightly forward in X
            targetPose = new Pose(targetPose.getX() + 3, targetPose.getY());
        }
        if (dpadLeft) {
            // Example: move turret target slightly backward in X
            targetPose = new Pose(targetPose.getX() - 3, targetPose.getY());
        }
    }

    public void resetOffset() {
        manualOffsetRad = 0.0;
    }

    public double getOffsetRadians() {
        return manualOffsetRad;
    }

    public double getOffsetDegrees() {
        return Math.toDegrees(manualOffsetRad);
    }

    // ================= IDLE =================
    public void idle() {
        double power =
            normalizePower(odometryPIDF.calculate(turretPosition, 0));
        yawMotor.setPower(power);
        limelight.pause();
    }

    public void stop() {
        yawMotor.setPower(0);
        limelight.stop();
    }

    public void resetEncoder() {
        yawMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    private double normalizePower(double power) {
        return Math.max(-1, Math.min(1, power));
    }

    // ================= GETTERS =================
    public double getTurretPosition() {
        return turretPosition;
    }

    public double getRelativeTargetHeading() {
        return relativeTargetHeading;
    }

    public double checkLL() {
        return isLLgetting;
    }
    public void syncToCurrentPosition() {
        turretPosition = yawMotor.getCurrentPosition() * RADIANSPERTICK;

        // Make PID think we are already at target
        relativeTargetHeading = turretPosition;

        manualOffsetRad = 0.0;
    }
    public void recalibration(Follower follower) {
        this.follower = follower;

        Pose currentPose = follower.getPose();

        double newX = currentPose.getX() - 13.0;
        double newY = currentPose.getY() - 12.0;

        targetPose = new Pose(newX, newY);
    }
}
