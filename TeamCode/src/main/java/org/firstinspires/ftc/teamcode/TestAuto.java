package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Tuning.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.acmerobotics.dashboard.FtcDashboard;


@Autonomous(name = "Test Auto", group = "Autonomous")
@Configurable
public class TestAuto extends OpMode {
    private DriveTrainHardware drive;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    public static double waitTime = 0.5;


    @Override
    public void init() {
        char ball1 = 'P';
        char ball2 = 'G';
        char ball3 = 'P';
        char detectedBall1 = 'P';
        char detectedBall2 = 'P';
        char detectedBall3 = 'G';
        char pos0 = detectedBall1;
        char pos1 = detectedBall2;
        char pos2 = detectedBall3;
        drive = new DriveTrainHardware();
        drive.init(hardwareMap);
        pathTimer = new Timer();
        opmodeTimer = new Timer();

    }

    @Override
    public void loop() {
        pathState = autonomousPathUpdate();
    }
        public void setPathState(int newState) {
            pathState = newState;
            pathTimer.resetTimer();
        }
        public int autonomousPathUpdate() {
        switch(pathState) {
            case 0:
                //Set spindexer to 90
                drive.intakeMotor.setPower(1);
                setPathState(1);
                break;
            case 1:
                if (pathTimer.seconds() > 2.0)


        }
        return pathState;
        }






}


