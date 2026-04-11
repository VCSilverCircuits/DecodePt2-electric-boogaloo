package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class PoseStorage {

    // Robot pose from auto
    public static Pose currentPose = new Pose(0, 0, 0);

    // Turret angle in radians
    public static double turretRadians = 0.0;
    public static double fingyDown = 0.05;
    public static double fingyUp = 1;

}
