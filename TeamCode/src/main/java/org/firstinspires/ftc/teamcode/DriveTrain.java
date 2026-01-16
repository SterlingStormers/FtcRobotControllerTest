package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
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
    private boolean kickerUp = false;
    private boolean kickerStart = false;
    private double kickerStartTime = 0.0;
    private boolean yWasPressed = false;
    private boolean spindexerMoving = false;
    private int targetPosition = 0;
    private final int COUNTS = 1365;
    private double SPIN_POWER = 0.15;
    private double currentSpin = 0;

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

//    public DriveTrain(HardwareMap hardwareMap) {
//    }

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
//        odom = hardwareMap.get(SensorGoBildaPinpoint.class, "Odom");

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

        ElapsedTime shooterTimer = new ElapsedTime();
        boolean shooterRunning = false;

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            int currentPos = intakeMotor.getCurrentPosition();
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
                if (gamepad2.y) {
                    spindexer.setPower(SPIN_POWER);
                } else {
                    spindexer.setPower(0);
                }
            } else {
                if (gamepad2.y && !yWasPressed) {
                    int startPos = currentPos;
                    if (SPIN_POWER > 0) {
                        targetPosition = startPos + COUNTS;
                    } else {
                        targetPosition = startPos - COUNTS;
                    }
                    currentSpin += 1;
                    if (currentSpin >= 8) {
                        currentSpin = 0;
                    }
                    spindexer.setPower(SPIN_POWER);
                    spindexerMoving = true;
                }

                yWasPressed = gamepad2.y;

                if (spindexerMoving) {
                    if (SPIN_POWER > 0) {
                        if (currentPos >= targetPosition) {
                            spindexer.setPower(0);
                            spindexerMoving = false;
                        } else if (currentPos >= targetPosition - 100) {
                            spindexer.setPower(0.18);
                            spindexerMoving = true;
                        }
                    } else {
                        if (currentPos <= targetPosition) {
                            spindexer.setPower(0);
                            spindexerMoving = false;
                        }
                    }
                }
            }

            if (gamepad2.a && !kickerUp && !kickerStart) {
                kickerStartTime = runtime.seconds();
                kickerStart = true;
            }

            if (kickerStart && (runtime.seconds() - kickerStartTime >= 1.0)) {
                kicker.setPosition(0.4);
                kickerUp = true;
                kickerStart = false;
                kickerStartTime = runtime.seconds();
            }

            if (kickerUp && (runtime.seconds() - kickerStartTime >= 1.5)) {
                kicker.setPosition(1);
                kickerUp = false;
            }

            if (gamepad2.a) {
                shooterMotor.setPower(1);
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