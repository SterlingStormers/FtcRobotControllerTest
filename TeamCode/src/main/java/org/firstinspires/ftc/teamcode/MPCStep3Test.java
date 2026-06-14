package org.firstinspires.ftc.teamcode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "MPC Step 3 Tier 2 Test", group = "Test")
public class MPCStep3Test extends LinearOpMode {
    @Override
    public void runOpMode() {
        DriveTrainHardware drive = new DriveTrainHardware();
        drive.init(hardwareMap);
        Follower follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        AMPC mpc = new AMPC(follower);
        VelocityControllerV2 controller = new VelocityControllerV2(follower, mpc);
        MecanumKinematics kinematics = new MecanumKinematics(drive, mpc, controller);

        PathChain testPath = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(0, 0), new Pose(50, 0)))
                .setLinearHeadingInterpolation(0, 0)
                .build();

        mpc.setActivePath(testPath);

        telemetry.addLine("MPC Step 3 Tier 2 — robot will drive itself.");
        telemetry.addLine("KEEP A HAND ON THE STOP BUTTON. No terminal cost yet — will overshoot end.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            follower.updatePose();
            mpc.update();
            controller.velocity();
            kinematics.drive();

            Pose pose = follower.getPose();
            telemetry.addData("robot", "(" + pose.getX() + ", " + pose.getY() + ", " + Math.toDegrees(pose.getHeading()) + "°)");
            telemetry.addData("currentT", mpc.currentT);
            telemetry.addData("desired V (x,y,ω)", "(" + mpc.desiredVx + ", " + mpc.desiredVy + ", " + mpc.desiredOmega + ")");
            telemetry.addData("actual V (x,y,ω)", "(" + controller.actualVx + ", " + controller.actualVy + ", " + controller.actualOmega + ")");
            telemetry.update();
        }
    }
}
