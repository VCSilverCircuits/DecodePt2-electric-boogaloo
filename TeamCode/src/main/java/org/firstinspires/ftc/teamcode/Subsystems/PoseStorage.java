package org.firstinspires.ftc.teamcode.Subsystems;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

public class PoseStorage {
    public static Pose currentPose = new Pose(0, 0, 0);
    public PoseStorage(Follower follower){
        this.follower = follower;

    }

    Follower follower;
    public void Update(){
        currentPose = follower.getPose();
    }
}
