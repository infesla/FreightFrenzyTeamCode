package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name="AutoBlueDuckStorage", group="AutoRed")
//@Disabled
public class AutoBlueDuckStorage extends LinearOpMode {

    public AutoMethods bot = new AutoMethods();

    //Стили езды: PID, continue, element

    @Override
    public void runOpMode() throws InterruptedException {

        bot.initC(this);
        bot.camStart(this, "blue");

        waitForStart();

        bot.camStop();
        bot.start();
        bot.Odometry.start();

        bot.getPos("blue");

        bot.markerReady();

        if (bot.baza == 1) {
            bot.GoTo(-14,28,0,0.35,2,4,1,"PID");
            bot.markerPick();
            bot.s1.setPosition(0.8);
            sleep(300);
        }
        if (bot.baza == 2) {
            bot.GoTo(0,28,0,0.35,2,4,1,"PID");
            bot.markerPick();
            bot.s1.setPosition(0.8);
            sleep(400);
        }
        if (bot.baza == 3) {
            bot.GoTo(14,28,0,0.35,2,4,1,"PID");
            bot.markerPick();
            bot.s1.setPosition(0.8);
            sleep(600);
        }

        //Приехал к хабу
        bot.GoTo(-15, 44,45,0.6,2, 2, bot.baza,"PID");

        //Сбросил элемент №1
        bot.s1.setPosition(0.4);
        sleep(500);
        bot.s1.setPosition(1);

        bot.GoTo(5, -4, 90, 0.6, 2, 2, 1, "PID");

        bot.m7.setPower(0.015);
        //Подъехал к колесу с уткой
        bot.GoTo(33,-6,90,0.3,1, 5.5,1,"PID");
        bot.m7.setPower(0);

        //Собрал утку с пола
        bot.GoTo(5,6,0,0.6,2, 5,1,"duck");

        //Приехал к хабу
        bot.GoTo(-15, 44,45,0.6,2, 2, 3,"PID");

        //Сбросил утку
        bot.s1.setPosition(0.4);
        sleep(500);
        bot.s1.setPosition(1);

        //Уехал в квадрат
        bot.GoTo(48,45,0,0.6,2, 3,1,"PID");

        sleep(1750);

        bot.GoTo(5, -4, 90, 0.7, 5, 3, 1, "PID");

        //Уехал на склад
        bot.GoTo(-140, -4, 90, 0.8, 2, 3, 1, "PID");

        //Запись угла в файл
        bot.writeAngle();
    }
}