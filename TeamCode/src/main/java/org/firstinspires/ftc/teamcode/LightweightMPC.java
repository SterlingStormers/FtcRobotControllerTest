package org.firstinspires.ftc.teamcode;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.follower.Follower;






public class LightweightMPC {
    private double baseForward;
    private double baseStrafe;
    private double baseTurn;
    private final Follower follower;
    private final DriveTrainHardware drive;
    public LightweightMPC(Follower follower, DriveTrainHardware drive) {
        this.follower = follower;
        this.drive = drive;
    }

    public void update() {
        // This is where the pulling happens.
        double currentX = follower.getPose().getX();
        double currentY = follower.getPose().getY();
        double currentHeading = follower.getPose().getHeading();
        double fl = drive.frontLeftDrive .getPower();
        double bl = drive.backLeftDrive  .getPower();
        double fr = drive.frontRightDrive.getPower();
        double br = drive.backRightDrive .getPower();

        double baseForward = (fl + bl + fr + br) / 4.0;
        double baseStrafe  = (fl - bl - fr + br) / 4.0;
        double baseTurn    = (fl + bl - fr - br) / 4.0;

        // velocity, path, target also pulled here

        // run the 125-candidate loop, score them, find the best

        // mix and send to motors at the end
        drive.frontLeftDrive.setPower();
        drive.backLeftDrive.setPower();
        drive.frontRightDrive.setPower();
        drive.backRightDrive.setPower();
    }
}