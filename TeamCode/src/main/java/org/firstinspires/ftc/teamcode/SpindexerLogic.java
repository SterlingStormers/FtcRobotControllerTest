package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name = "Spindexer Logic", group = "Autonomous")
@Configurable
public class SpindexerLogic extends OpMode {
    private DriveTrainHardware drive;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    public static double waitTime = 0.5;
    private char ball1 = 'P';
    private char ball2 = 'G';
    private char ball3 = 'P';
    private char detectedBall1 = 'P';
    private char detectedBall2 = 'P';
    private char detectedBall3 = 'G';
    private char pos0 = detectedBall1;
    private char pos1 = detectedBall2;
    private char pos2 = detectedBall3;
    private boolean slot0 = false;
    private boolean slot1 = false;
    private boolean slot2 = false;


    @Override
    public void init() {
        drive = new DriveTrainHardware();
        drive.init(hardwareMap);
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        DcMotor motor = hardwareMap.dcMotor.get("intake_motor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done
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
                drive.intakeMotor.setPower(1);
                setPathState(1);
                break;
            case 1:
                if (pathTimer.getElapsedTimeSeconds() >= waitTime) {
                    if (drive.intakeMotor.getCurrentPosition() < 2731 - 150) {
                        drive.spindexer.setPower(0.5);
                    } else if (drive.intakeMotor.getCurrentPosition() < 2731) {
                        drive.spindexer.setPower(0.18);
                    } else {
                        drive.spindexer.setPower(0);
                        // trigger camera
                        slot0 = true;
                        setPathState(2);
                    }
                }
                break;
            case 2:
                if (pathTimer.getElapsedTimeSeconds() >= waitTime) {
                    if (drive.intakeMotor.getCurrentPosition() < 5462 - 150) {
                        drive.spindexer.setPower(0.5);
                    } else if (drive.intakeMotor.getCurrentPosition() < 5462) {
                        drive.spindexer.setPower(0.18);
                    } else {
                        drive.spindexer.setPower(0);
                        // trigger camera
                        slot1 = true;
                        setPathState(3);
                    }
                }
                break;
            case 3:
                // 360° target = 8192 counts
                if (pathTimer.getElapsedTimeSeconds() >= waitTime) {
                    if (drive.intakeMotor.getCurrentPosition() < 8192 - 150) {
                        drive.spindexer.setPower(0.5);
                    } else if (drive.intakeMotor.getCurrentPosition() < 8192) {
                        drive.spindexer.setPower(0.18);
                    } else {
                        drive.spindexer.setPower(0);
                        // trigger camera
                        slot2 = true;
                        setPathState(4);
                    }
                }
                break;
            case 4:
                drive.intakeMotor.setPower(0);
                drive.shooterMotor.setPower(1);
                    setPathState(5);

            break;
            case 5:





        }
        return pathState;
        }






}


