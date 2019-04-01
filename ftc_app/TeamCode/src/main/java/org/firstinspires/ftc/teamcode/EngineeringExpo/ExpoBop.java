package org.firstinspires.ftc.teamcode.EngineeringExpo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.bot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


@TeleOp(name="EXPO COOL",group="Teleop")
public class ExpoBop extends OpMode {
    botExpo bot = new botExpo();

    @Override
    public void init() {
        bot.init(hardwareMap, telemetry, false);
    }
    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    public void loop() {

    bot.drive(gamepad1.left_stick_y, gamepad1.right_stick_y);

    if(gamepad1.right_trigger > .3){
        bot.FlyL.setPower(1);
        bot.FlyR.setPower(1);
    }


    }


}
