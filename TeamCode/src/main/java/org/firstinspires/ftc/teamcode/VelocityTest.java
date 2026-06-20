package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;

@TeleOp(name="Velocity Test", group="Test")
public class VelocityTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        Follower follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        waitForStart();

        double lastX = 0;
        double lastY = 0;
        long lastTime = System.nanoTime();

        while (opModeIsActive()) {

            follower.updatePose();

            Pose p = follower.getPose();

            long now = System.nanoTime();
            double dt = (now - lastTime) / 1e9;
            lastTime = now;

            double vx = (p.getX() - lastX) / dt;
            double vy = (p.getY() - lastY) / dt;

            lastX = p.getX();
            lastY = p.getY();

            telemetry.addData("X", p.getX());
            telemetry.addData("Y", p.getY());
            telemetry.addData("VX", vx);
            telemetry.addData("VY", vy);
            telemetry.update();
        }
    }
}