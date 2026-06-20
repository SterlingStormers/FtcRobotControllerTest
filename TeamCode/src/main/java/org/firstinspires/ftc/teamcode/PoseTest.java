package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Pose Stability Test", group = "Test")
public class PoseTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        Follower follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        waitForStart();

        Pose lastPose = follower.getPose();
        long lastTime = System.nanoTime();

        while (opModeIsActive()) {

            follower.update();

            Pose p = follower.getPose();

            long now = System.nanoTime();
            double dt = (now - lastTime) / 1e9;
            lastTime = now;

            double dx = p.getX() - lastPose.getX();
            double dy = p.getY() - lastPose.getY();

            double vx = dx / dt;
            double vy = dy / dt;

            lastPose = p;

            telemetry.addLine("=== POSE ===");
            telemetry.addData("X", p.getX());
            telemetry.addData("Y", p.getY());
            telemetry.addData("Heading", Math.toDegrees(p.getHeading()));

            telemetry.addLine("=== DELTA PER LOOP ===");
            telemetry.addData("dX", dx);
            telemetry.addData("dY", dy);

            telemetry.addLine("=== VELOCITY ===");
            telemetry.addData("Vx", vx);
            telemetry.addData("Vy", vy);

            telemetry.addLine("=== STABILITY CHECK ===");
            telemetry.addData("Pose Drift Magnitude", Math.sqrt(dx*dx + dy*dy));

            telemetry.update();
        }
    }
}