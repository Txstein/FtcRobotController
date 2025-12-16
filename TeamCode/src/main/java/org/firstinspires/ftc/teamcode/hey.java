package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class hey extends OpMode {
    @Override
    public void init(){

    }

    @Override
    public void init_loop(){
        telemetry.addData("hello","?");
    }

    @Override
    public void loop(){

    }
}
