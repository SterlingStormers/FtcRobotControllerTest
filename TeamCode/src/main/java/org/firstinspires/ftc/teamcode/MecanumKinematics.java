package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * @author Sahaj Patel - 23345 Sterling Stormers
 * @version 1.0, 7/17/2026
 */

public class MecanumKinematics {
    private final DcMotor frontLeftDrive;
    private final DcMotor frontRightDrive;
    private final DcMotor backLeftDrive;
    private final DcMotor backRightDrive;
    private final AMPC ampc;
    private final VelocityControllerV2 controller;

    public MecanumKinematics(HardwareMap hardwareMap, AMPC ampc, VelocityControllerV2 controller) {
        this.ampc = ampc;
        this.controller = controller;

        frontLeftDrive = hardwareMap.get(DcMotor.class, SlipstreamConstants.leftFrontMotorName);
        frontRightDrive = hardwareMap.get(DcMotor.class, SlipstreamConstants.rightFrontMotorName);
        backLeftDrive = hardwareMap.get(DcMotor.class, SlipstreamConstants.leftBackMotorName);
        backRightDrive = hardwareMap.get(DcMotor.class, SlipstreamConstants.rightBackMotorName);

        frontLeftDrive.setDirection(SlipstreamConstants.leftFrontDirection);
        frontRightDrive.setDirection(SlipstreamConstants.rightFrontDirection);
        backLeftDrive.setDirection(SlipstreamConstants.leftBackDirection);
        backRightDrive.setDirection(SlipstreamConstants.rightBackDirection);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void drive() {
        double vxNorm = controller.effortVx / ampc.maxSpeedForward;
        double vyNorm = controller.effortVy / ampc.maxSpeedStrafe;
        double omegaNorm = controller.effortOmega / ampc.maxTurnRateRad;

        double fl = vxNorm - vyNorm - omegaNorm;
        double fr = vxNorm + vyNorm + omegaNorm;
        double bl = vxNorm + vyNorm - omegaNorm;
        double br = vxNorm - vyNorm + omegaNorm;

        double maxWheel = Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br))));
        if (maxWheel > 1.0) {
            fl /= maxWheel;
            fr /= maxWheel;
            bl /= maxWheel;
            br /= maxWheel;
        }

        frontLeftDrive.setPower(fl);
        frontRightDrive.setPower(fr);
        backLeftDrive.setPower(bl);
        backRightDrive.setPower(br);
    }
}