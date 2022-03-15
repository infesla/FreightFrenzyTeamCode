package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name="AutoRedDuckStorage", group="AutoRed")
//@Disabled
public class AutoRedDuckStorage extends LinearOpMode {

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

        bot.markerReady();

        if (bot.baza == 1) {
            bot.GoTo(-9,30,0,0.35,2,4,1,"PID");
            bot.markerPick();
            bot.s1.setPosition(0.8);
            sleep(300);
        }
        if (bot.baza == 2) {
            bot.GoTo(5,30,0,0.35,2,4,1,"PID");
            bot.markerPick();
            bot.s1.setPosition(0.8);
            sleep(400);
        }
        if (bot.baza == 3) {
            bot.GoTo(17,30,0,0.35,2,4,1,"PID");
            bot.markerPick();
            bot.s1.setPosition(0.8);
            sleep(600);
        }

        //Приехал к хабу
        bot.GoTo(26, 37,-32,0.6,1, 3, bot.baza,"PID");

        //Сбросил элемент №1
        bot.s1.setPosition(0.4);
        sleep(500);
        bot.s1.setPosition(1);

        bot.m7.setPower(-0.015);
        //Подъехал к колесу с уткой
        bot.GoTo(-20,-7,-90,0.5,1, 2,1,"PID");
        bot.GoTo(-30,-10,-90,0.4,1, 5.5,1,"PID");
        bot.m7.setPower(0);
        bot.GoTo(-20,0,0,0.25,1, 1,1,"PID");
        bot.GoTo(-20,10,0,0.3,1, 1,1,"PID");
        bot.GoTo(-9,10,0,0.3,1, 1,1,"PID");
        bot.GoTo(-9,3,0,0.4,1, 6,1,"duck");

        //Приехал к хабу
        bot.GoTo(26, 37,-32,0.6,1, 3, 3,"PID");

        //Сбросил утку
        bot.s1.setPosition(0.4);
        sleep(500);
        bot.s1.setPosition(1);

        bot.GoTo(28, -6,90,0.6,1, 2, 1,"PID");
        bot.GoTo(140, -7,90,0.6,1, 4, 1,"PID");

        //Запись угла в файл
        bot.writeAngle();


    }
}