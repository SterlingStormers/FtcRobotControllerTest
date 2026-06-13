package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "MPC Step 1 Test", group = "Test")
public class MPCStep1Test extends LinearOpMode {
    @Override
    public void runOpMode() {
        Follower follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        AMPC mpc = new AMPC(follower);

        // Build a simple test path: line from (0,0) to (50, 0)
        PathChain testPath = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(0, 0), new Pose(50, 0)))
                .setLinearHeadingInterpolation(0, 0)
                .build();

        mpc.setActivePath(testPath);

        telemetry.addLine("MPC Step 1 Test ready");
        telemetry.addLine("Manually push the robot along the test line (0,0) to (50,0)");
        telemetry.addLine("Watch t-value increase as you move along the line");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            follower.updatePose();
            mpc.update();

            Pose currentPose = follower.getPose();
            telemetry.addData("robot X", currentPose.getX());
            telemetry.addData("robot Y", currentPose.getY());
            Pose midSample = mpc.getActivePath().getPath(0).getPose(0.5);
            telemetry.addData("path midpoint", "(" + midSample.getX() + ", " + midSample.getY() + ")");
            telemetry.addData("currentT", mpc.currentT);
            telemetry.addData("lookaheadT", mpc.lookaheadT);
            telemetry.addData("lookahead X", mpc.lookaheadPose.getX());
            telemetry.addData("lookahead Y", mpc.lookaheadPose.getY());
            telemetry.addData("desiredVx", mpc.desiredVx);
            telemetry.addData("desiredVy", mpc.desiredVy);
            telemetry.addData("desiredOmega", mpc.desiredOmega);
            telemetry.update();
        }
    }
}