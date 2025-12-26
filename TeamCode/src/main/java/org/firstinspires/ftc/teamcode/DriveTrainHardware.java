package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
        frontLeftDrive = hw.get(DcMotorEx.class, "front_left_drive");
        backLeftDrive = hw.get(DcMotorEx.class, "back_left_drive");
        frontRightDrive = hw.get(DcMotorEx.class, "front_right_drive");
        backRightDrive = hw.get(DcMotorEx.class, "back_right_drive");
        intakeMotor = hw.get(DcMotorEx.class, "intake_motor");
        shooterMotor = hw.get(DcMotorEx.class, "shooter_motor");
        spindexer = hw.get(CRServo.class, "spindexer_servo");

        //To be changed

        frontLeftDrive.setDirection(DcMotorEx.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorEx.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorEx.Direction.FORWARD);
        backRightDrive.setDirection(DcMotorEx.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
        shooterMotor.setDirection(DcMotorEx.Direction.REVERSE);
        spindexer.setDirection(DcMotorSimple.Direction.FORWARD);

        //To be changed
    }
}
