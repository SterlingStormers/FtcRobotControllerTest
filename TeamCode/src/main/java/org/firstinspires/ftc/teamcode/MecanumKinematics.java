package org.firstinspires.ftc.teamcode;

public class MecanumKinematics {
    private final DriveTrainHardware drive;
    private final AMPC mpc;
    private final VelocityControllerV2 controller;
    public MecanumKinematics(DriveTrainHardware drive, AMPC mpc, VelocityControllerV2 controller) {
        this.drive = drive;
        this.mpc = mpc;
        this.controller = controller;
    }
    public void drive() {
        // Normalize using MPC's current model parameters
        double vxNorm = controller.effortVx / mpc.maxSpeedForward;
        double vyNorm = controller.effortVy / mpc.maxSpeedStrafe;
        double omegaNorm = controller.effortOmega / mpc.maxTurnRateRad;
        // Mecanum mixing
        double fl = vxNorm + vyNorm + omegaNorm;
        double fr = vxNorm - vyNorm - omegaNorm;
        double bl = vxNorm - vyNorm + omegaNorm;
        double br = vxNorm + vyNorm - omegaNorm;
        // Scale if any wheel exceeds 1.0
        double maxWheel = Math.max(Math.abs(fl),
                Math.max(Math.abs(fr),
                        Math.max(Math.abs(bl), Math.abs(br))));
        if (maxWheel > 1.0) {
            fl /= maxWheel;
            fr /= maxWheel;
            bl /= maxWheel;
            br /= maxWheel;
        }
        // Write motors
        drive.frontLeftDrive.setPower(fl);
        drive.frontRightDrive.setPower(fr);
        drive.backLeftDrive.setPower(bl);
        drive.backRightDrive.setPower(br);
    }
}
