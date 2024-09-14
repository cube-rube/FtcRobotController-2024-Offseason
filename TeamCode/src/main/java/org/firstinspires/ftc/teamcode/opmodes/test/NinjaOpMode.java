package org.firstinspires.ftc.teamcode.opmodes.test;

import com.ftc9929.corelib.control.NinjaGamePad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class NinjaOpMode extends LinearOpMode {
    private NinjaGamePad gamePad1;
    private NinjaGamePad gamePad2;
    @Override
    public void runOpMode() throws InterruptedException {
        gamePad1 = new NinjaGamePad(gamepad1);
        gamePad2 = new NinjaGamePad(gamepad2);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("dpad_up", gamePad1.getDpadUp().debounced().getRise());
            telemetry.addData("dpad_down", gamePad1.getDpadDown().debounced().getRise());
            telemetry.update();
        }
    }
}
