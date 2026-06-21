package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Hold Position Test", group = "Test")
public class PoseTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        DriveTrainHardware drive = new DriveTrainHardware();
        drive.init(hardwareMap);

        Follower follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        AMPC mpc = new AMPC(follower);   // only used for desiredV* fields
        VelocityControllerV2 controller = new VelocityControllerV2(follower, mpc);
        MecanumKinematics kinematics = new MecanumKinematics(drive, mpc, controller);

        // Hold gains (P-only)
        double KP_POS = 5;
        double KP_HEADING = 5.0;

        waitForStart();

        while (opModeIsActive()) {
            follower.updatePose();
            Pose p = follower.getPose();

            // Field-frame error (target = origin)
            double errorXField = -p.getX();
            double errorYField = -p.getY();

            // Rotate field error into robot frame for the velocity command
            double heading = p.getHeading();
            double cosH = Math.cos(heading);
            double sinH = Math.sin(heading);
            double cmdVx = (errorXField * cosH + errorYField * sinH) * KP_POS;
            double cmdVy = (-errorXField * sinH + errorYField * cosH) * KP_POS;
            double cmdOmega = -KP_HEADING * heading;

            mpc.desiredVx = cmdVx;
            mpc.desiredVy = cmdVy;
            mpc.desiredOmega = cmdOmega;

            controller.velocity();
            kinematics.drive();

            telemetry.addData("X (localizer)", p.getX());
            telemetry.addData("Y (localizer)", p.getY());
            telemetry.addData("heading deg", Math.toDegrees(heading));
            telemetry.addData("desired Vx/Vy/Omega",
                    "(" + cmdVx + ", " + cmdVy + ", " + cmdOmega + ")");
            telemetry.update();
        }
    }
}