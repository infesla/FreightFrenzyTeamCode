package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="AutoExperiment", group="Test")
//@Disabled
public class AutoExperiment extends LinearOpMode {

    public AutoMethods bot = new AutoMethods();

    //Стили езды: PID, continue, element

    @Override
    public void runOpMode() throws InterruptedException {

        bot.initC(this);
        bot.camStart(this, "red");

        waitForStart();

        bot.camStop();
        bot.start();
        bot.Odometry.start();

        bot.getPos("red");

        //Едет задом на 40 и ищет элемент
        bot.GoTo(0,-40,0,0.3,10, 5,1,"element");
    }
}