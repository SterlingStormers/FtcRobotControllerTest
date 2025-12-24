package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrainHardware {
    public DcMotor frontLeftDrive;
    public DcMotor backLeftDrive;
    public DcMotor frontRightDrive;
    public DcMotor backRightDrive;
    public DcMotor intakeMotor;
    public DcMotor shooterMotor;
    public CRServo spindexer;

    public void init(HardwareMap hw) {
        frontLeftDrive = hw.get(DcMotor.class, "front_left_drive");
        backLeftDrive = hw.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hw.get(DcMotor.class, "front_right_drive");
        backRightDrive = hw.get(DcMotor.class, "back_right_drive");
        intakeMotor = hw.get(DcMotor.class, "intake_motor");
        shooterMotor = hw.get(DcMotor.class, "shooter_motor");
        spindexer = hw.get(CRServo.class, "spindexer_servo");
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
        kicker.setDirection(DcMotorSimple.Direction.FORWARD);
        hood.setDirection(DcMotorSimple.Direction.FORWARD);
        //To be changed
    }
}
