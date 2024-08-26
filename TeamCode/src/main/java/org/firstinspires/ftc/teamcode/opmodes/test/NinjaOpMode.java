package org.firstinspires.ftc.teamcode.opmodes.test;

import com.ftc9929.corelib.control.NinjaGamePad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class NinjaOpMode extends LinearOpMode {
    private final NinjaGamePad gamePad1 = new NinjaGamePad(gamepad1);
    private final NinjaGamePad gamePad2 = new NinjaGamePad(gamepad2);
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("dpad_up", gamePad1.getDpadUp().debounced().getRise());
            telemetry.addData("dpad_down", gamePad1.getDpadDown().debounced().getRise());
        }
    }
}
