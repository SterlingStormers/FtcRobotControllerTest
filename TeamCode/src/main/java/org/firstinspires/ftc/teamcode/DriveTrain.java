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

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;


@TeleOp

public class DriveTrain extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor frontLeftDrive = null;
    public DcMotor backLeftDrive = null;
    public DcMotor frontRightDrive = null;
    public DcMotor backRightDrive = null;
    public DcMotor intakeMotor = null;
    public DcMotor shooterMotor = null;
    public CRServo spindexer = null;
    public Servo servo0 = null;
    public Servo servo1 = null;
    public Servo servo2 = null;
    public Servo servo3 = null;
    public CRServo servo4 = null;

    public DriveTrain(HardwareMap hardwareMap) {
    }

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
        kicker = hardwareMap.get(servoX.class, "kicker_servo");
        hood = hardwareMap.get(servoX.class, "hood_servo");
        //To be changed

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        shooterMotor.setDirection(DcMotor.Direction.REVERSE);
        spindexer.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        ElapsedTime shooterTimer = new ElapsedTime();
        boolean shooterRunning = false;

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double modifier = 1;

            if (gamepad1.left_bumper) {
                modifier = 0.5;
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

            double spinPower = 0.35;

            if (gamepad2.left_bumper) {
                spinPower = -0.35;
            }

            // spindexer (CHANGE LATER)
            if (gamepad2.y) {
                spindexer.setPower(spinPower);
            } else {
                spindexer.setPower(0);
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