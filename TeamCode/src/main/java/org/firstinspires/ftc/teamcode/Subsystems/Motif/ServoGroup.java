package org.firstinspires.ftc.teamcode.Subsystems.Motif;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Subsystems.ColorSensorTests.ColorSensors;

import java.util.ArrayList;
import java.util.List;

public class ServoGroup {

    private final Servo s1, s2, s3, stopper;

    private final List<Servo> firingOrder = new ArrayList<>();

    private int index = 0;
    private boolean servoUp = false;
    private long lastTime = 0;
    private boolean running = false;

    private Servo lastAddedServo = null;

    // Timing constants
    private static final long UP_DURATION_MS = 220;
    private static final long DOWN_DELAY_MS = 220;

    public ServoGroup(HardwareMap hw,
                      String servo1, String servo2, String servo3, String stopper1) {

        stopper = hw.get(Servo.class,stopper1);
        s1 = hw.get(Servo.class, servo1);
        s2 = hw.get(Servo.class, servo2);
        s3 = hw.get(Servo.class, servo3);
        s1.setPosition(0.05);
        s2.setPosition(0.05);
        s3.setPosition(0.05);

        stop();
    }

    /* =========================================================
       🔫 FORCED SEQUENTIAL MODE (s1 → s2 → s3)
       ========================================================= */
    public void StartNonSort() {
        running = true;
        stopper.setPosition(0.3);
        firingOrder.clear();
        index = 0;
        servoUp = false;
        running = true;
        lastTime = System.currentTimeMillis();
        lastAddedServo = null;

        addServoSafely(s1);
        addServoSafely(s2);
        addServoSafely(s3);
    }
    public void idle(){
        stopper.setPosition(0);
        s1.setPosition(0.05);
        s2.setPosition(0.05);
        s3.setPosition(0.05);
    }

    /* =========================================================
       🎼 MOTIF-BASED MODE
       ========================================================= */
    public void startMotif(MatchMotif.MotifPattern motif, ColorSensors sensors) {

        firingOrder.clear();
        index = 0;
        servoUp = false;
        running = true;
        lastTime = System.currentTimeMillis();
        lastAddedServo = null;

        buildFiringOrder(motif, sensors);

        // Absolute safety fallback
        if (firingOrder.isEmpty()) {
            addServoSafely(s1);
            addServoSafely(s2);
            addServoSafely(s3);
        }
    }

    private void buildFiringOrder(MatchMotif.MotifPattern motif, ColorSensors sensors) {

        ColorSensors.DetectedColor[] pattern;

        switch (motif) {
            case GPP:
                pattern = new ColorSensors.DetectedColor[]{
                    ColorSensors.DetectedColor.GREEN,
                    ColorSensors.DetectedColor.PURPLE,
                    ColorSensors.DetectedColor.PURPLE
                };
                break;
            case PGP:
                pattern = new ColorSensors.DetectedColor[]{
                    ColorSensors.DetectedColor.PURPLE,
                    ColorSensors.DetectedColor.GREEN,
                    ColorSensors.DetectedColor.PURPLE
                };
                break;
            case PPG:
                pattern = new ColorSensors.DetectedColor[]{
                    ColorSensors.DetectedColor.PURPLE,
                    ColorSensors.DetectedColor.PURPLE,
                    ColorSensors.DetectedColor.GREEN
                };
                break;
            default:
                pattern = new ColorSensors.DetectedColor[]{
                    ColorSensors.DetectedColor.GREEN,
                    ColorSensors.DetectedColor.PURPLE,
                    ColorSensors.DetectedColor.PURPLE
                };
                break;
        }

        for (ColorSensors.DetectedColor color : pattern) {
            Servo candidate = null;

            if (sensors.getS1() == color) candidate = s1;
            else if (sensors.getS2() == color) candidate = s2;
            else if (sensors.getS3() == color) candidate = s3;

            if (candidate != null) {
                addServoSafely(candidate);
            }
        }
    }

    /* =========================================================
       🛡 SAFETY: NO BACK-TO-BACK REPEATS
       ========================================================= */
    private void addServoSafely(Servo servo) {
        if (servo != lastAddedServo) {
            firingOrder.add(servo);
            lastAddedServo = servo;
        }
    }

    /* =========================================================
       ⏱ TIMING LOOP
       ========================================================= */
    public void loop() {

        // If not running, keep stopper closed and exit
        if (!running) {
            idle();
            return;
        }

        stopper.setPosition(0.3);

        // ✅ SEQUENCE FINISHED
        if (index >= firingOrder.size()) {
            running = false;
            stopper.setPosition(0);
            return;
        }

        long now = System.currentTimeMillis();
        Servo current = firingOrder.get(index);

        if (servoUp) {
            if (now - lastTime >= UP_DURATION_MS) {
                current.setPosition(0.05);
                servoUp = false;
                lastTime = now;
                index++;
            }
        } else {
            if (now - lastTime >= DOWN_DELAY_MS) {
                current.setPosition(0.8);
                servoUp = true;
                lastTime = now;
            }
        }

}

    public boolean isRunning() {
        return running && index < firingOrder.size();
    }

    public int getIndex() {
        return index;
    }

    public int getSequenceLength() {
        return firingOrder.size();
    }

    public void stop() {
        running = false;
        index = 0;
        servoUp = false;
        firingOrder.clear();
        lastAddedServo = null;

        s1.setPosition(0.05);
        s2.setPosition(0.05);
        s3.setPosition(0.05);
    }
}
