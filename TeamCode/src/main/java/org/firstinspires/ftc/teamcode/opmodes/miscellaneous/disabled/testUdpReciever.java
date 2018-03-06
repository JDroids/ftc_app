package org.firstinspires.ftc.teamcode.opmodes.miscellaneous.disabled;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.resources.external.PidUdpReceiver;

import java.math.RoundingMode;
import java.text.DecimalFormat;

/**
 * Created by dansm on 2/28/2018.
 */
@Disabled
@TeleOp(name = "Udp Reciever Test")

public class testUdpReciever extends LinearOpMode {
    private double p, i, d;
    private PidUdpReceiver pidUdpReceiver;

    @Override
    public void runOpMode() {
        pidUdpReceiver = new PidUdpReceiver();
        pidUdpReceiver.beginListening();

        telemetry.setMsTransmissionInterval(50);
        waitForStart();

        while (opModeIsActive()) {
            updateCoefficients();

            telemetry.addData("P", formatVal(p));
            telemetry.addData("I", formatVal(i));
            telemetry.addData("D", formatVal(d));
            telemetry.update();
        }

        pidUdpReceiver.shutdown();

    }

    private void updateCoefficients() {
        p = pidUdpReceiver.getP();
        i = pidUdpReceiver.getI();
        d = pidUdpReceiver.getD();
    }

    private String formatVal(double val) {
        DecimalFormat df = new DecimalFormat("#.###");
        df.setRoundingMode(RoundingMode.CEILING);
        return df.format(val);
    }


}
