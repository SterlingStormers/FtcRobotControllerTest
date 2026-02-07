package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


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
    private boolean slot0 = false;
    private boolean slot1 = false;
    private boolean slot2 = false;
    private DcMotor motor;
    private boolean kickerUp = false;
    private double kickerPos = 0;
    private double kickerStartTime = 0.0;
    public boolean has180Occured = false;
    private ElapsedTime runtime = new ElapsedTime();
    public int pos = 0;


    @Override
    public void init() {
        drive = new DriveTrainHardware();
        drive.init(hardwareMap);
        drive.kicker.setPosition(0);
        pos = drive.intakeMotor.getCurrentPosition();
        kickerPos = drive.kicker.getPosition();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
//        motor = hardwareMap.get(DcMotor.class, "intake_motor");
        drive.intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        drive.intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Makes sure intake motor does not rely on
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
                drive.intakeMotor.setPower(-1);
                setPathState(1);
                telemetry.addData("case", 0);
                break;
            case 1:
                telemetry.addData("case", 1);
                pos = drive.intakeMotor.getCurrentPosition();
                if (pathTimer.getElapsedTimeSeconds() >= waitTime) {
                    int remaining = 2731 - pos; //ccw
                    double power = 0;
                    power = (0.0005 * remaining);
                    power = Math.max(power, -1);
                    power = Math.min(power, 1);
                    if (Math.abs(remaining) <= 35) {
                        power = 0;
                        slot0 = true;
                        setPathState(2);
                    }
                    drive.spindexer.setPower(power);
                }
                break;
            case 2:
                telemetry.addData("case", 2);
                pos = drive.intakeMotor.getCurrentPosition();
                if (pathTimer.getElapsedTimeSeconds() >= waitTime) {
                    int remaining = 5462 - pos; //ccw
                    double power = 0;
                    power = (0.0005 * remaining);
                    power = Math.max(power, -1);
                    power = Math.min(power, 1);
                    if (Math.abs(remaining) <= 35) {
                        power = 0;
                        slot1 = true;
                        setPathState(3);
                    }
                    drive.spindexer.setPower(power);
                }
                break;
            case 3:
                telemetry.addData("case", 3);
                pos = drive.intakeMotor.getCurrentPosition();
                if (pathTimer.getElapsedTimeSeconds() >= waitTime) {
                    int remaining = 8192 - pos; //ccw
                    double power = 0;
                    power = (0.0005 * Math.abs(remaining));
                    power = Math.max(power, -1);
                    power = Math.min(power, 1);
                    if (Math.abs(remaining) <= 35) {
                        telemetry.addData("caseReached", 4);
                        power = 0;
                        drive.spindexer.setPower(power);
                        slot2 = true;
                        setPathState(4);
                    }
                    telemetry.addData("remaining: ", remaining);
                    drive.spindexer.setPower(power);
                }
                break;
            case 4:
                telemetry.addData("case", 4);
                drive.intakeMotor.setPower(0);
                drive.shooterMotor.setPower(1);
                setPathState(5);
                break;
            case 5: {
                telemetry.addData("case", 5);
                pos = drive.intakeMotor.getCurrentPosition();
                if (detectedBall3 == ball1) { // detectedBall3 is at slot2
                    int remaining = 9557 - pos; //ccw
                    double power = 0;
                    power = (0.0005 * remaining);
                    power = Math.max(power, -1);
                    power = Math.min(power, 1);
                    if (Math.abs(remaining) <= 35) {
                        power = 0;
                    }
                    if (Math.abs(remaining) >= 100) {
                        pathTimer.resetTimer();
                    }
                    telemetry.addData("Timer", pathTimer.getElapsedTimeSeconds());
                    drive.spindexer.setPower(power);
                    if (pathTimer.getElapsedTimeSeconds() >= 0.3) {
                        drive.spindexer.setPower(0);
                        if (!kickerUp) {
                            drive.kicker.setPosition(kickerPos + 1);
                            telemetry.addData("kickerUp", true);
                            if (drive.kicker.getPosition() > kickerPos + 0.3) {
                                kickerUp = true;
                                kickerStartTime = runtime.seconds();
                            }
                        }
                        if (kickerUp && (runtime.seconds() - kickerStartTime) >= 0.5) {
                            drive.kicker.setPosition(kickerPos);
                            kickerUp = false;
                            slot2 = false;
                            setPathState(6);
                        }
                    }

//--------------------------------USE FOR TIMER-----------------------------------
//                    if (Math.abs(remaining) <= 35) {
//                        power = 0;
//                    }
//                    if (Math.abs(remaining) >= 100) {
//                        pathTimer.resetTimer();
//                    }
//                    telemetry.addData("Timer", pathTimer.getElapsedTimeSeconds());
//
//                    if (pathTimer.getElapsedTimeSeconds() >= 0.3) {

                } else if (detectedBall2 == ball1) { // detectedBall2 is at slot1
                    int remaining = pos - 6827; //cw
                    double power = 0;
                    power = (-0.0005 * remaining);
                    power = Math.max(power, -1);
                    power = Math.min(power, 1);

                    if (Math.abs(remaining) <= 35) {
                        power = 0;
                    }
                    if (Math.abs(remaining) >= 100) {
                        pathTimer.resetTimer();
                    }
                    telemetry.addData("Timer", pathTimer.getElapsedTimeSeconds());
                    drive.spindexer.setPower(power);

                     if (pathTimer.getElapsedTimeSeconds() >= 0.3) {
                        drive.spindexer.setPower(0);
                        if (!kickerUp) {
                            drive.kicker.setPosition(kickerPos + 1);
                            telemetry.addData("kickerUp", true);
                            if (drive.kicker.getPosition() > kickerPos + 0.3) {
                                kickerUp = true;
                                kickerStartTime = runtime.seconds();
                            }
                        }
                        if (kickerUp && (runtime.seconds() - kickerStartTime) >= 0.5) {
                            drive.kicker.setPosition(kickerPos);
                            kickerUp = false;
                            slot1 = false;
                            setPathState(6);
                        }
                    }
                } else if (detectedBall1 == ball1) { //detectedBall1 is at slot0
                    int remaining = 12288 - pos; //ccw
                    double power = 0;
                    power = (0.0005 * remaining);
                    power = Math.max(power, -1);
                    power = Math.min(power, 1);
                    if (Math.abs(remaining) <= 35) {
                        power = 0;
                    }
                    if (Math.abs(remaining) >= 100) {
                        pathTimer.resetTimer();
                    }
                    telemetry.addData("Timer", pathTimer.getElapsedTimeSeconds());
                    drive.spindexer.setPower(power);
                    if (pathTimer.getElapsedTimeSeconds() >= 0.3) {
                        drive.spindexer.setPower(0);
                        if (!kickerUp) {
                            drive.kicker.setPosition(kickerPos + 1);
                            telemetry.addData("kickerUp", true);
                            if (drive.kicker.getPosition() > kickerPos + 0.3) {
                                kickerUp = true;
                                kickerStartTime = runtime.seconds();
                            }
                        }
                        if (kickerUp && (runtime.seconds() - kickerStartTime) >= 0.5) {
                            drive.kicker.setPosition(kickerPos);
                            kickerUp = false;
                            has180Occured = true;
                            slot0 = false;
                            setPathState(6);
                        }
                    }
                } else {
                    telemetry.addLine("There has been an error with the amount of balls expected. Continuing auto");
                    telemetry.update();
                    setPathState(6);
                }
                break;
            }
            case 6:
                telemetry.addData("case", 6);
                pos = drive.intakeMotor.getCurrentPosition();
                if (detectedBall3 == ball2 && slot2 && has180Occured) { // detectedBall3 is at slot2
                    int remaining = pos - 9557; //cw
                    double power = 0;
                    power = (-0.0005 * remaining);
                    power = Math.max(power, -1);
                    power = Math.min(power, 1);
                    if (Math.abs(remaining) <= 35) {
                        power = 0;
                    }
                    if (Math.abs(remaining) >= 100) {
                        pathTimer.resetTimer();
                    }
                    telemetry.addData("Timer", pathTimer.getElapsedTimeSeconds());
                    drive.spindexer.setPower(power);
                    if (pathTimer.getElapsedTimeSeconds() >= 0.3) {
                        drive.spindexer.setPower(0);
                        if (!kickerUp) {
                            drive.kicker.setPosition(kickerPos + 1);
                            telemetry.addData("kickerUp", true);
                            if (drive.kicker.getPosition() > kickerPos + 0.3) {
                                kickerUp = true;
                                kickerStartTime = runtime.seconds();
                            }
                        }
                        if (kickerUp && (runtime.seconds() - kickerStartTime) >= 0.5) {
                            drive.kicker.setPosition(kickerPos);
                            kickerUp = false;
                            has180Occured = false;
                            slot2 = false;
                            setPathState(7);
                        }
                    }
                } else if (detectedBall3 == ball2 && slot2 && !has180Occured) { // detectedBall3 is at slot2
                    int remaining = 9557 - pos; //ccw
                    double power = 0;
                    power = (0.0005 * remaining);
                    power = Math.max(power, -1);
                    power = Math.min(power, 1);
                    if (Math.abs(remaining) <= 35) {
                        power = 0;
                    }
                    if (Math.abs(remaining) >= 100) {
                        pathTimer.resetTimer();
                    }
                    telemetry.addData("Timer", pathTimer.getElapsedTimeSeconds());
                    drive.spindexer.setPower(power);
                    if (pathTimer.getElapsedTimeSeconds() >= 0.3) {
                        drive.spindexer.setPower(0);
                        if (!kickerUp) {
                            drive.kicker.setPosition(kickerPos + 1);
                            telemetry.addData("kickerUp", true);
                            if (drive.kicker.getPosition() > kickerPos + 0.3) {
                                kickerUp = true;
                                kickerStartTime = runtime.seconds();
                            }
                        }
                        if (kickerUp && (runtime.seconds() - kickerStartTime) >= 0.5) {
                            drive.kicker.setPosition(kickerPos);
                            kickerUp = false;
                            slot2 = false;
                            setPathState(7); //15019
                        }
                    }
                } else if (detectedBall2 == ball2 && slot1 && has180Occured) { //detectedBall2 is at slot1
                    int remaining = 15019 - pos; //ccw
                    double power = 0;
                    power = (0.0005 * remaining);
                    power = Math.max(power, -1);
                    power = Math.min(power, 1);
                    if (Math.abs(remaining) <= 35) {
                        power = 0;
                    }
                    if (Math.abs(remaining) >= 100) {
                        pathTimer.resetTimer();
                    }
                    telemetry.addData("Timer", pathTimer.getElapsedTimeSeconds());
                    drive.spindexer.setPower(power);
                    if (pathTimer.getElapsedTimeSeconds() >= 0.3) {
                        drive.spindexer.setPower(0);
                        if (!kickerUp) {
                            drive.kicker.setPosition(kickerPos + 1);
                            telemetry.addData("kickerUp", true);
                            if (drive.kicker.getPosition() > kickerPos + 0.3) {
                                kickerUp = true;
                                kickerStartTime = runtime.seconds();
                            }
                        }
                        if (kickerUp && (runtime.seconds() - kickerStartTime) >= 0.5) {
                            drive.kicker.setPosition(kickerPos);
                            kickerUp = false;
                            has180Occured = false;
                            slot1 = false;
                            setPathState(7);
                        }
                    }
                } else if (detectedBall2 == ball2 && slot1 && !has180Occured) { //detectedBall2 is at slot1
                    int remaining = 6827 - pos; //ccw
                    double power = 0;
                    power = (0.0005 * remaining);
                    power = Math.max(power, -1);
                    power = Math.min(power, 1);
                    if (Math.abs(remaining) <= 35) {
                        power = 0;
                    }
                    if (Math.abs(remaining) >= 100) {
                        pathTimer.resetTimer();
                    }
                    telemetry.addData("Timer", pathTimer.getElapsedTimeSeconds());
                    drive.spindexer.setPower(power);
                    if (pathTimer.getElapsedTimeSeconds() >= 0.3) {
                        drive.spindexer.setPower(0);
                        if (!kickerUp) {
                            drive.kicker.setPosition(kickerPos + 1);
                            telemetry.addData("kickerUp", true);
                            if (drive.kicker.getPosition() > kickerPos + 0.3) {
                                kickerUp = true;
                                kickerStartTime = runtime.seconds();
                            }
                        }
                        if (kickerUp && (runtime.seconds() - kickerStartTime) >= 0.5) {
                            drive.kicker.setPosition(kickerPos);
                            kickerUp = false;
                            slot1 = false;
                            setPathState(7);
                        }
                    }
                } else if (detectedBall1 == ball2 && slot0) { //detectedBall1 is at slot0
                    int remaining = 12288 - pos; //ccw
                    double power = 0;
                    power = (0.0005 * remaining);
                    power = Math.max(power, -1);
                    power = Math.min(power, 1);
                    if (Math.abs(remaining) <= 35) {
                        power = 0;
                    }
                    if (Math.abs(remaining) >= 100) {
                        pathTimer.resetTimer();
                    }
                    telemetry.addData("Timer", pathTimer.getElapsedTimeSeconds());
                    drive.spindexer.setPower(power);
                    if (pathTimer.getElapsedTimeSeconds() >= 0.3) {
                        drive.spindexer.setPower(0);
                        if (!kickerUp) {
                            drive.kicker.setPosition(kickerPos + 1);
                            telemetry.addData("kickerUp", true);
                            if (drive.kicker.getPosition() > kickerPos + 0.3) {
                                kickerUp = true;
                                kickerStartTime = runtime.seconds();
                            }
                        }
                        if (kickerUp && (runtime.seconds() - kickerStartTime) >= 0.5) {
                            drive.kicker.setPosition(kickerPos);
                            kickerUp = false;
                            has180Occured = true;
                            slot0 = false;
                            setPathState(7);
                        }
                    }
                } else {
                    telemetry.addLine("There has been an error with the amount of balls expected. Continuing auto");
                    telemetry.update();
                    setPathState(7);
                }
                break;
            case 7:
                telemetry.addData("case", 7);
                pos = drive.intakeMotor.getCurrentPosition();
                if (detectedBall3 == ball3 && slot2 && has180Occured) { // detectedBall3 is at slot2
                    int remaining = pos - 9557; //cw
                    double power = 0;
                    power = (-0.0005 * remaining);
                    power = Math.max(power, -1);
                    power = Math.min(power, 1);
                    if (Math.abs(remaining) <= 35) {
                        power = 0;
                    }
                    if (Math.abs(remaining) >= 100) {
                        pathTimer.resetTimer();
                    }
                    telemetry.addData("Timer", pathTimer.getElapsedTimeSeconds());
                    drive.spindexer.setPower(power);
                    if (pathTimer.getElapsedTimeSeconds() >= 0.3) {
                        drive.spindexer.setPower(0);
                        if (!kickerUp) {
                            drive.kicker.setPosition(kickerPos + 1);
                            telemetry.addData("kickerUp", true);
                            if (drive.kicker.getPosition() > kickerPos + 0.3) {
                                kickerUp = true;
                                kickerStartTime = runtime.seconds();
                            }
                        }
                        if (kickerUp && (runtime.seconds() - kickerStartTime) >= 0.5) {
                            drive.kicker.setPosition(kickerPos);
                            kickerUp = false;
                            has180Occured = false;
                            slot2 = false;
                            setPathState(8);
                        }
                    }
                } else if (detectedBall3 == ball3 && slot2 && !has180Occured) { // detectedBall3 is at slot2
                    int remaining = 9557 - pos; //ccw
                    double power = 0;
                    power = (0.0005 * remaining);
                    power = Math.max(power, -1);
                    power = Math.min(power, 1);
                    if (Math.abs(remaining) <= 35) {
                        power = 0;
                    }
                    if (Math.abs(remaining) >= 100) {
                        pathTimer.resetTimer();
                    }
                    telemetry.addData("Timer", pathTimer.getElapsedTimeSeconds());
                    drive.spindexer.setPower(power);
                    if (pathTimer.getElapsedTimeSeconds() >= 0.3) {
                        drive.spindexer.setPower(0);
                        if (!kickerUp) {
                            drive.kicker.setPosition(kickerPos + 1);
                            telemetry.addData("kickerUp", true);
                            if (drive.kicker.getPosition() > kickerPos + 0.3) {
                                kickerUp = true;
                                kickerStartTime = runtime.seconds();
                            }
                        }
                        if (kickerUp && (runtime.seconds() - kickerStartTime) >= 0.5) {
                            drive.kicker.setPosition(kickerPos);
                            kickerUp = false;
                            slot2 = false;
                            setPathState(8); //15019
                        }
                    }
                } else if (detectedBall2 == ball3 && slot1 && has180Occured) { //detectedBall2 is at slot1
                    int remaining = 15019 - pos; //ccw
                    double power = 0;
                    power = (0.0005 * remaining);
                    power = Math.max(power, -1);
                    power = Math.min(power, 1);
                    if (Math.abs(remaining) <= 35) {
                        power = 0;
                    }
                    if (Math.abs(remaining) >= 100) {
                        pathTimer.resetTimer();
                    }
                    telemetry.addData("Timer", pathTimer.getElapsedTimeSeconds());
                    drive.spindexer.setPower(power);
                    if (pathTimer.getElapsedTimeSeconds() >= 0.3) {
                        drive.spindexer.setPower(0);
                        if (!kickerUp) {
                            drive.kicker.setPosition(kickerPos + 1);
                            telemetry.addData("kickerUp", true);
                            if (drive.kicker.getPosition() > kickerPos + 0.3) {
                                kickerUp = true;
                                kickerStartTime = runtime.seconds();
                            }
                        }
                        if (kickerUp && (runtime.seconds() - kickerStartTime) >= 0.5) {
                            drive.kicker.setPosition(kickerPos);
                            kickerUp = false;
                            has180Occured = false;
                            slot1 = false;
                            setPathState(8);
                        }
                    }
                } else if (detectedBall2 == ball3 && slot1 && !has180Occured) { //detectedBall2 is at slot1
                    int remaining = 6827 - pos; //ccw
                    double power = 0;
                    power = (0.0005 * remaining);
                    power = Math.max(power, -1);
                    power = Math.min(power, 1);
                    if (Math.abs(remaining) <= 35) {
                        power = 0;
                    }
                    if (Math.abs(remaining) >= 100) {
                        pathTimer.resetTimer();
                    }
                    telemetry.addData("Timer", pathTimer.getElapsedTimeSeconds());
                    drive.spindexer.setPower(power);
                    if (pathTimer.getElapsedTimeSeconds() >= 0.3) {
                        drive.spindexer.setPower(0);
                        if (!kickerUp) {
                            drive.kicker.setPosition(kickerPos + 1);
                            telemetry.addData("kickerUp", true);
                            if (drive.kicker.getPosition() > kickerPos + 0.3) {
                                kickerUp = true;
                                kickerStartTime = runtime.seconds();
                            }
                        }
                        if (kickerUp && (runtime.seconds() - kickerStartTime) >= 0.5) {
                            drive.kicker.setPosition(kickerPos);
                            kickerUp = false;
                            slot1 = false;
                            setPathState(8);
                        }
                    }
                } else if (detectedBall1 == ball3 && slot0) { //detectedBall1 is at slot0
                    int remaining = 12288 - pos; //ccw
                    double power = 0;
                    power = (0.0005 * remaining);
                    power = Math.max(power, -1);
                    power = Math.min(power, 1);
                    if (Math.abs(remaining) <= 35) {
                        power = 0;
                    }
                    if (Math.abs(remaining) >= 100) {
                        pathTimer.resetTimer();
                    }
                    telemetry.addData("Timer", pathTimer.getElapsedTimeSeconds());
                    drive.spindexer.setPower(power);
                    if (pathTimer.getElapsedTimeSeconds() >= 0.3) {
                        drive.spindexer.setPower(0);
                        if (!kickerUp) {
                            drive.kicker.setPosition(kickerPos + 1);
                            telemetry.addData("kickerUp", true);
                            if (drive.kicker.getPosition() > kickerPos + 0.3) {
                                kickerUp = true;
                                kickerStartTime = runtime.seconds();
                            }
                        }
                        if (kickerUp && (runtime.seconds() - kickerStartTime) >= 0.5) {
                            drive.kicker.setPosition(kickerPos);
                            kickerUp = false;
                            slot0 = false;
                            setPathState(8);
                        }
                    }
                } else {
                    telemetry.addLine("There has been an error with the amount of balls expected. Continuing auto");
                    telemetry.update();
                    setPathState(8);
                }
                break;
            case 8:
                drive.shooterMotor.setPower(0);
                setPathState(-1);
        }
        return pathState;
        }
}


