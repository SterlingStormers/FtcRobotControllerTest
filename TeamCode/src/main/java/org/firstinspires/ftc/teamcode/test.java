package org.firstinspires.ftc.teamcode;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp (name = "test", group = "TeleOp")
@Configurable
public class test extends OpMode {
    private DriveTrainHardware drive;
    private DcMotor motor;

    @Override
    public void init() {
        drive = new DriveTrainHardware();
        drive.init(hardwareMap);
        motor = hardwareMap.dcMotor.get("intake_motor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    @Override
    public void loop() {
        int position = motor.getCurrentPosition();
        telemetry.addData("Encoder Position", position);
        telemetry.addLine("Rotate the encoder shaft by hand");
        telemetry.addLine("If CCW increases → CCW = positive");
        telemetry.addLine("If CCW decreases → CCW = negative");
        telemetry.update();
    }
}
