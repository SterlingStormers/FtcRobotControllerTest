package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Auto Bottom 6 Blue", group = "Autonomous")
@Configurable // Panels
public class AutoBottom6Blue extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private DriveTrainHardware drive;
    private Timer pathTimer, opmodeTimer;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
        drive = new DriveTrainHardware();
        drive.init(hardwareMap);
        pathTimer = new Timer();
        opmodeTimer = new Timer();
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.154, 7.488), new Pose(72.000, 72.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(72.000, 72.000), new Pose(59.220, 84.296))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(142))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(59.220, 84.296), new Pose(34.854, 84.135))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(34.854, 84.135), new Pose(16.459, 84.296))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(16.459, 84.296),
                                    new Pose(39.372, 85.587),
                                    new Pose(53.411, 90.105)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(142))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(53.411, 90.105), new Pose(65.029, 98.819))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(142))
                    .build();
        }
    }
    public void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    public int autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        double t = follower.getCurrentTValue();
        follower.update();
        switch(pathState) {
            case 0:
                drive.intakeMotor.setPower(0);
                drive.shooterMotor.setPower(0);
                follower.setMaxPower(1);
                setPathState(1);
                break;
            case 1:
                    follower.followPath(paths.Path1);
                    setPathState(2);
                break;
            case 2:
                if(!follower.isBusy()) {
                    //Tell HuskyLens to scan AprilTag
                    // HuskyLens will then send color pattern to ColorSensingAuto class for logic and this class for telemetry
                    // Do not block loop, use HuskyLens polling
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path2, true);
                }
                follower.update();
                drive.shooterMotor.setPower(1);
                //Color Sensing
                //Spindexer logic
                //Do not block loop
                if (!follower.isBusy()) {
                    setPathState(4);
                }
                break;
            case 4:
                //Spindexer logic
                //Shooter logic
                //Do not block loop
                setPathState(5);
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3, false);
                }

                follower.update();
                if (follower.getCurrentTValue() >= 0.5) {
                    drive.intakeMotor.setPower(1);
                    drive.shooterMotor.setPower(0);
                }
                if(!follower.isBusy()){
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.5);
                    follower.followPath(paths.Path4, false);
                }
                follower.update();
                //Run Color Sensing
                //Run Spindexer
                //Do not block loop
                if ((!follower.isBusy())){
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.setMaxPower(1);
                    follower.followPath(paths.Path5, true);
                }
                follower.update();
                if (follower.getCurrentTValue() >= 0.5){
                    drive.intakeMotor.setPower(0);
                    drive.shooterMotor.setPower(1);
                    drive.kicker.setPosition(0.5);
                }
                //Spindexer logic
                //Do not block loop
                if(!follower.isBusy()) {
                    setPathState(8);
                }
                break;
            case 8:
                //Spindexer logic
                //Shooter logic
                //Do not block loop
                setPathState(9);
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path6, true);
                }
                follower.update();
                if (follower.getCurrentTValue() >= 0.5) {
                    drive.kicker.setPosition(0);
                    drive.shooterMotor.setPower(0);
                }
                if (!follower.isBusy() && pathState != -1) {
                    telemetry.addLine("Successfully completed 6 ball auto");
                    telemetry.update();
                    drive.frontLeftDrive.setPower(0);
                    drive. backLeftDrive.setPower(0);
                    drive.frontRightDrive.setPower(0);
                    drive.backRightDrive.setPower(0);
                    drive.intakeMotor.setPower(0);
                    drive.shooterMotor.setPower(0);
                    drive.spindexer.setPower(0);

                    //To be changed
                    pathState = -1;
                }
        }
        return pathState;
    }
}
