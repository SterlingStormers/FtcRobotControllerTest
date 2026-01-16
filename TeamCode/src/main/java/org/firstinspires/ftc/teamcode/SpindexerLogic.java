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
    public int cwORcww = 1;
    public boolean has180Occured = false;
    private ElapsedTime runtime = new ElapsedTime();
    private boolean braking = false;
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
        if (has180Occured == true) {
         cwORcww = -1;
        }
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
                if (pathTimer.getElapsedTimeSeconds() >= waitTime) {
                    if (drive.intakeMotor.getCurrentPosition() <= 2731 - 200) {  //120
                        drive.spindexer.setPower(1);
                    } else if ((drive.intakeMotor.getCurrentPosition() >= (2731 - 200)) && (drive.intakeMotor.getCurrentPosition() < 2731)) {
                        drive.spindexer.setPower(0.18);
                    } else if (drive.intakeMotor.getCurrentPosition() >= 2731) {
                        drive.spindexer.setPower(0);
                        // trigger camera
                        slot0 = true;
                        setPathState(2);
                    }
                }
                telemetry.addData("case", 1);
                break;
            case 2:
                if (pathTimer.getElapsedTimeSeconds() >= waitTime) {
                    if (drive.intakeMotor.getCurrentPosition() < 5462 - 200) {  //240
                        drive.spindexer.setPower(1);
                    } else if (drive.intakeMotor.getCurrentPosition() < 5462) {
                        drive.spindexer.setPower(0.18);
                    } else {
                        drive.spindexer.setPower(0);
                        // trigger camera
                        slot1 = true;
                        setPathState(3);
                    }
                }
                telemetry.addData("case", 2);
                break;
            case 3:
                if (pathTimer.getElapsedTimeSeconds() >= waitTime) {
                    if (drive.intakeMotor.getCurrentPosition() < 8192 - 200) {  //360
                        drive.spindexer.setPower(1);
                    } else if (drive.intakeMotor.getCurrentPosition() < 8192) {
                        drive.spindexer.setPower(0.18);
                    } else {
                        drive.spindexer.setPower(0);
                        // trigger camera
                        slot2 = true;
                        setPathState(4);
                    }
                }
                telemetry.addData("case", 3);
                break;
            case 4:
                drive.intakeMotor.setPower(0);
                drive.shooterMotor.setPower(1);
                telemetry.addData("case", 4);
                setPathState(5);
                break;
            case 5: {
                telemetry.addData("case", 5);
                int pos = drive.intakeMotor.getCurrentPosition();
                telemetry.addData("intakePos", pos);

                // --- slot2 at goal (original branch) ---
                if (detectedBall3 == ball1) {
                    if (pos < 9557 - 200) {
                        drive.spindexer.setPower(1);
                    } else if (pos < 9557) {
                        drive.spindexer.setPower(0.18);
                    } else {
                        drive.spindexer.setPower(0);
                        if (!kickerUp) {
                            telemetry.addData("kickerUp3", true);
                            drive.kicker.setPosition(kickerPos + 1);
                            kickerUp = true;
                            kickerStartTime = runtime.seconds();
                        }
                        if (kickerUp && (runtime.seconds() - kickerStartTime) >= 2.0) {
                            drive.kicker.setPosition(kickerPos);
                            kickerUp = false;
                            slot2 = false;
                            setPathState(6);
                        }
                    }


                } else if (detectedBall2 == ball1) {
                    int remaining = pos - 6827;
                    if (remaining >= 500) {
                        drive.spindexer.setPower(-1);
                    } else if (remaining >= 50) {
                        drive.spindexer.setPower(-0.18);
                    }
                     if (!braking && remaining <= 50) {
                        drive.spindexer.setPower(0.1);
                        telemetry.addData("braking", true);
                        braking = true;
                        pathTimer.resetTimer();
                    }
                     if (braking && pathTimer.getElapsedTimeSeconds() >= 0.020) {
                        drive.spindexer.setPower(0);

                        if (!kickerUp) {
                            drive.kicker.setPosition(kickerPos + 1);
                            telemetry.addData("kickerUp2", true);
                            kickerUp = true;
                            kickerStartTime = runtime.seconds();
                        }
                        if (kickerUp && (runtime.seconds() - kickerStartTime) >= 0.5) {
                            drive.kicker.setPosition(kickerPos);
                            kickerUp = false;
                            braking = false;
                            slot1 = false;
                            setPathState(6);
                        }
                    }

                    // --- slot0 at goal (original branch) ---
                } else if (detectedBall1 == ball1) {
                    if (pos < 12288 - 200) {
                        drive.spindexer.setPower(1);
                    } else if (pos < 12288) {
                        drive.spindexer.setPower(0.18);
                    } else {
                        drive.spindexer.setPower(0);
                        if (!kickerUp) {
                            drive.kicker.setPosition(kickerPos + 1);
                            telemetry.addData("kickerUp1", true);
                            kickerUp = true;
                            kickerStartTime = runtime.seconds();
                        }
                        if (kickerUp && (runtime.seconds() - kickerStartTime) >= 2.0) {
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
                //---
                if (detectedBall3 == ball2 && slot2 && has180Occured) { // detectedBall3 is at slot2
                    telemetry.addData("Scenario", 4);
                    if (pos >= 9557 + 200) { //420 degrees
                        drive.spindexer.setPower(-1);
                    } else if (pos >= 9557 + 50 && pos < 9557 + 250) {
                        drive.spindexer.setPower(-0.18);
                    } else if (!braking && pos >= 9557 && pos < 9557 + 50) {
                        drive.spindexer.setPower(0.1);
                        telemetry.addData("Braking?", true);
                        braking = true;
                        pathTimer.resetTimer();
                    } if (braking && pathTimer.getElapsedTimeSeconds() >= 0.05) {
                        drive.spindexer.setPower(0);
                        braking = false;
                        if (!kickerUp) {
                            drive.kicker.setPosition(kickerPos + 1);
                            telemetry.addData("kickerUp", true);
                            kickerUp = true;
                            kickerStartTime = runtime.seconds();
                        } if (kickerUp && (runtime.seconds() - kickerStartTime) >= 1.0) {
                            drive.kicker.setPosition(kickerPos);
                            kickerUp = false;
                            has180Occured = false;
                            slot2 = false;
                            setPathState(7);
                        }
                    }
                } else if (detectedBall3 == ball2 && slot2 && !has180Occured) { // detectedBall3 is at slot2
                    telemetry.addData("Scenario",5 );
                    if (pos <= 9557 - 200) { //420 degrees
                        drive.spindexer.setPower(1);
                    } else if (pos <= 9557 - 50 && pos > 9557 - 250) {
                        drive.spindexer.setPower(0.18);
                    } else if (!braking && pos <= 9557 && pos > 9557 - 50) {
                        drive.spindexer.setPower(-0.1);
                        telemetry.addData("Braking?", true);
                        braking = true;
                        pathTimer.resetTimer();
                    }if (braking && pathTimer.getElapsedTimeSeconds() >= 0.05) {
                        drive.spindexer.setPower(0);
                        braking = false;
                        if (!kickerUp) {
                            drive.kicker.setPosition(kickerPos + 1);
                            telemetry.addData("kickerUp", true);
                            kickerUp = true;
                            kickerStartTime = runtime.seconds();
                        } if (kickerUp && (runtime.seconds() - kickerStartTime) >= 1.0) {
                            drive.kicker.setPosition(kickerPos);
                            kickerUp = false;
                            slot2 = false;
                            setPathState(7);
                        }
                    }
                    //---
                } else if (detectedBall2 == ball2 && slot1 && has180Occured) { //detectedBall2 is at slot1
                    telemetry.addData("Scenario", 6);
                    if (pos <= 15019 - 200) { //660
                        drive.spindexer.setPower(1);
                    } else if (pos <= 15019 - 50 && pos > 15019 - 250) {
                        drive.spindexer.setPower(0.18);
                    } else if (!braking && pos <= 15019 && pos > 15019 - 50) {
                            drive.spindexer.setPower(-0.1);
                            telemetry.addData("Braking?", true);
                            braking = true;
                            pathTimer.resetTimer();
                        } if (braking && pathTimer.getElapsedTimeSeconds() >= 0.05) {
                            drive.spindexer.setPower(0);
                            braking = false;
                            if (!kickerUp) {
                                drive.kicker.setPosition(kickerPos + 1);
                                telemetry.addData("kickerUp", true);
                                kickerUp = true;
                                kickerStartTime = runtime.seconds();
                            } if (kickerUp && (runtime.seconds() - kickerStartTime) >= 1.0) {
                                drive.kicker.setPosition(kickerPos);
                                kickerUp = false;
                                has180Occured = false;
                                slot1 = false;
                                setPathState(7);
                            }
                    }
                } else if (detectedBall2 == ball2 && slot1 && !has180Occured) { //detectedBall2 is at slot1
                    telemetry.addData("Scenario", 7);
                    if (pos >= 6827 + 200) { //300 degrees
                        drive.spindexer.setPower(-1);
                    } else if (pos >= 6827 + 50 && pos < 6827 + 250) {
                        drive.spindexer.setPower(-0.18);
                    } else if (!braking && pos >= 6827 && pos < 6827 + 50) {
                        drive.spindexer.setPower(0.1);
                        telemetry.addData("Braking?", true);
                        braking = true;
                        pathTimer.resetTimer();
                    }
                    if (braking && pathTimer.getElapsedTimeSeconds() >= 0.05) {
                        drive.spindexer.setPower(0);
                        braking = false;
                        if (!kickerUp) {
                            drive.kicker.setPosition(kickerPos + 1);
                            telemetry.addData("kickerUp", true);
                            kickerUp = true;
                            kickerStartTime = runtime.seconds();
                        } if (kickerUp && (runtime.seconds() - kickerStartTime) >= 1.0) {
                            drive.kicker.setPosition(kickerPos);
                            kickerUp = false;
                            slot1 = false;
                            setPathState(7);
                        }
                    } // ---
                } else if (detectedBall1 == ball2 && slot0) { //detectedBall1 is at slot0
                    if (drive.intakeMotor.getCurrentPosition() < 12288 - 200) {
                        drive.spindexer.setPower(1);
                    } else if (drive.intakeMotor.getCurrentPosition() < 12288) {
                        drive.spindexer.setPower(0.18);
                    } else {
                        drive.spindexer.setPower(0);
                        if (!kickerUp) {
                            drive.kicker.setPosition(kickerPos + 1);
                            kickerUp = true;
                            kickerStartTime = runtime.seconds();
                        }
                        if (kickerUp && (runtime.seconds() - kickerStartTime) >= 2.0) {
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
                if (detectedBall3 == ball3 && slot2) { // detectedBall3 is at slot2
                    if (drive.intakeMotor.getCurrentPosition() < 10923 - 200) {
                        drive.spindexer.setPower(cwORcww * 1);
                    } else if (drive.intakeMotor.getCurrentPosition() < 10923) {
                        drive.spindexer.setPower(cwORcww * 0.18);
                    } else {
                        drive.spindexer.setPower(0);
                        if (!kickerUp) {
                            drive.kicker.setPosition(kickerPos + 1);
                            kickerUp = true;
                            kickerStartTime = runtime.seconds();
                        }
                        if (kickerUp && (runtime.seconds() - kickerStartTime) >= 2.0) {
                            drive.kicker.setPosition(kickerPos);
                            kickerUp = false;
                            has180Occured = false;
                            slot2 = false;
                            setPathState(8);
                        }
                    }
                } else if (detectedBall2 == ball3 && slot1) { //detectedBall2 is at slot1
                    if (drive.intakeMotor.getCurrentPosition() < 5462 - 200) {
                        drive.spindexer.setPower(cwORcww * -1);
                    } else if (drive.intakeMotor.getCurrentPosition() < 5462) {
                        drive.spindexer.setPower(cwORcww * -0.18);
                    } else {
                        drive.spindexer.setPower(0);
                        if (!kickerUp) {
                            drive.kicker.setPosition(kickerPos + 1);
                            kickerUp = true;
                            kickerStartTime = runtime.seconds();
                        }
                        if (kickerUp && (runtime.seconds() - kickerStartTime) >= 2.0) {
                            drive.kicker.setPosition(kickerPos);
                            kickerUp = false;
                            has180Occured = false;
                            slot1 = false;
                            setPathState(8);
                        }
                    }
                } else if (detectedBall1 == ball3 && slot0) { //detectedBall1 is at slot0
                    if (drive.intakeMotor.getCurrentPosition() < 12288 - 200) {
                        drive.spindexer.setPower(1);
                    } else if (drive.intakeMotor.getCurrentPosition() < 12288) {
                        drive.spindexer.setPower(0.18);
                    } else {
                        drive.spindexer.setPower(0);
                        if (!kickerUp) {
                            drive.kicker.setPosition(kickerPos + 1);
                            kickerUp = true;
                            kickerStartTime = runtime.seconds();
                        }
                        if (kickerUp && (runtime.seconds() - kickerStartTime) >= 2.0) {
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


