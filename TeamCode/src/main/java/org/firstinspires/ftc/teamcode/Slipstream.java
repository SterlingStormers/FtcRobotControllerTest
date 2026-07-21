package org.firstinspires.ftc.teamcode;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * @author Sahaj Patel - 23345 Sterling Stormers
 * @version 1.0, 7/20/2026
 */

public class Slipstream {
    public Follower follower;

    public AMPC ampc;
    public VelocityControllerV2 controller;
    public MecanumKinematics kinematics;

    public Slipstream(Follower follower, HardwareMap hardwareMap) {
        this.follower = follower;
        ampc = new AMPC(follower);
        controller = new VelocityControllerV2(follower, ampc);
        kinematics = new MecanumKinematics(hardwareMap, ampc, controller);
    }

    public void update() {
        follower.updatePose();

        // Sync AMPC with whatever path the user told Pedro to follow
        PathChain currentChain = follower.getCurrentPathChain();
        if (currentChain != null && currentChain != ampc.getActivePath()) {
            ampc.setActivePath(currentChain);
        }

        ampc.update();
        controller.velocity();
        kinematics.drive();

        // When AMPC finishes, tell Pedro so !follower.isBusy() works
        if (ampc.isPathComplete() && follower.isBusy()) {
            follower.breakFollowing();
        }
    }
}