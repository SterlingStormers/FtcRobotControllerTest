package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorGoBildaPinpoint;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;

@TeleOp

public class DriveTrain extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private Timer pathTimer, opmodeTimer;
    private boolean kickerUp = false;
    private boolean kickerStart = false;
    private double kickerStartTime = 0.0;
    private boolean yWasPressed = false;
    private boolean spindexerMoving = false;
    private int COUNTS = 1365;
    private int offsetCounts = 0;
    private int comparableThreshold = 0;
    private double SPIN_POWER = -0.2;
    private int currentSpin = 0;

    public DcMotor frontLeftDrive = null;
    public DcMotor backLeftDrive = null;
    public DcMotor frontRightDrive = null;
    public DcMotor backRightDrive = null;
    public DcMotor intakeMotor = null;
    public DcMotor shooterMotor = null;
    public CRServo spindexer = null;
    public SensorGoBildaPinpoint odom = null;
    public Servo kicker = null;
    private double kickerPos = 0;
    public Servo servo0 = null;
    public Servo servo1 = null;
    public Servo servo2 = null;
    public Servo servo3 = null;
    public CRServo servo4 = null;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        spindexer = hardwareMap.get(CRServo.class, "spindexer_servo");
        shooterMotor = hardwareMap.get(DcMotor.class, "shooter_motor");
        kicker = hardwareMap.get(Servo.class, "kicker_servo");

        //To be changed

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        spindexer.setDirection(CRServo.Direction.FORWARD);
        kicker.setDirection(Servo.Direction.REVERSE);

        //To be changed

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        kicker.setPosition(1);
        kickerPos = kicker.getPosition();

        pathTimer = new Timer();
        opmodeTimer = new Timer();

        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ElapsedTime shooterTimer = new ElapsedTime();
        boolean shooterRunning = false;

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double modifier = 1;

            if (gamepad1.left_bumper) {
                modifier = 0.5;
            }
            if (gamepad1.right_bumper) {
                modifier = 1.5;
            }

            double max;

            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            double frontLeftPower = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower = axial - lateral + yaw;
            double backRightPower = axial + lateral - yaw;

            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }

            // intake
            if (gamepad2.x) {
                intakeMotor.setPower(1);
            } else {
                intakeMotor.setPower(0);
            }

            if (gamepad2.left_bumper) {
                COUNTS = 2729;
            } else {
                COUNTS = 1365;
            }
            if (gamepad2.y && !yWasPressed && !spindexerMoving && !gamepad2.left_bumper) {
                if (SPIN_POWER > 0) {
                    currentSpin += COUNTS;
                } else {
                    currentSpin -= COUNTS;
                }
                spindexerMoving = true;
                pathTimer.resetTimer();
            }

            yWasPressed = gamepad2.y;

            if (spindexerMoving) {
               // int remaining = targetPosition - intakeMotor.getCurrentPosition(); //ccw
                int remaining = currentSpin - intakeMotor.getCurrentPosition();
                double power = 0;
                power = (0.0005 * remaining);
                power = Math.max(power, -1);
                power = Math.min(power, 1);

                int tolerance = 30;
                if (comparableThreshold > 0) {
                    tolerance = comparableThreshold;
                }

//                if (Math.abs(remaining) <= tolerance) {
//                    power = 0;
//                }
//                if (Math.abs(remaining) >= tolerance) {
//                    pathTimer.resetTimer();
//                }

                spindexer.setPower(power);
                telemetry.addData("remaining: ", remaining);

                double timeoutSec = 0.85;
                if (Math.abs(remaining) <= tolerance && pathTimer.getElapsedTimeSeconds() >= timeoutSec) {
//                    offsetCounts = remaining
                    spindexer.setPower(0);
                    spindexerMoving = false;
                } else if (pathTimer.getElapsedTimeSeconds() >= timeoutSec) {
                    telemetry.addData("timed out: ", true);
//                    offsetCounts = remaining;
                    spindexer.setPower(0);
                    spindexerMoving = false;
                }
            }

            if (gamepad2.a && !kickerUp && !kickerStart) {
                kickerStartTime = runtime.seconds();
                kickerStart = true;
            }

            if (kickerStart && (runtime.seconds() - kickerStartTime >= 2) && (shooterMotor.getPower() >= 0.5)) {
                kicker.setPosition(0.25);
                kickerUp = true;
                kickerStart = false;
                kickerStartTime = runtime.seconds();
            }

            if (kickerUp && (runtime.seconds() - kickerStartTime >= 0.75)) {
                kicker.setPosition(1);
                kickerUp = false;
            }

            if (gamepad2.a) {
                shooterMotor.setPower(0.9);

            } else {
                shooterMotor.setPower(0);
            }

            frontLeftDrive.setPower(frontLeftPower * modifier);
            frontRightDrive.setPower(frontRightPower * modifier);
            backLeftDrive.setPower(backLeftPower * modifier);
            backRightDrive.setPower(backRightPower * modifier);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
            telemetry.update();
        }
    }
}