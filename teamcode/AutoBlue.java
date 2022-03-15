package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="AutoBlue", group="AutoBlue")
//@Disabled
public class AutoBlue extends LinearOpMode {

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
            bot.GoTo(-2,33,50,0.3,2,4,1,"PID");
            bot.markerPick();
            bot.s1.setPosition(0.8);
            bot.GoTo(0,38,-60,0.35,2,2,1,"PID");
        }
        if (bot.baza == 2) {
            bot.GoTo(1,30,0,0.35,1,4,1,"PID");
            bot.markerPick();
            bot.s1.setPosition(0.8);
            sleep(300);
        }
        if (bot.baza == 3) {
            bot.GoTo(15,30,0,0.35,2,4,1,"PID");
            bot.markerPick();
            bot.s1.setPosition(0.8);
            sleep(500);
        }

        //Приехал к хабу
        bot.GoTo(15, 40,-43,0.6,2, 2, bot.baza,"PID");

        //Сбросил элемент №1
        bot.s1.setPosition(0.4);
        sleep(500);
        bot.s1.setPosition(1);

        //Подъехал к въезду в склад и запросил калибровку координат
        bot.GoTo(0,-6,-90,0.6,3, 3,1,"PID");
        bot.GoTo(0,-6,-90,0.4,1, 1,1,"PID");
        bot.obnul(0.02, -31, -6); // обнуление координат

        //Заехал в склад
        bot.GoTo(-45,-12,-90,0.6,10, 3,1,"PID");

        //Взял элемент №2
        bot.GoTo(-65,-8,-90,0.4,10, 4,1,"element");

        //Подъехал к выезду со склада и запросил калибровку координат
        bot.GoTo(-40,-9,-90,0.6,5,2,1,"PID");
        bot.obnul(0.02, -31, -6); // обнуление координат

        //Выехал со склада
        bot.GoTo(0,-9,-90,0.5,5, 2,1,"PID");

        //Преднаклон корзины
        bot.s1.setPosition(0.8);

        //Подъехал к хабу
        bot.GoTo(15, 40,-43,0.6,2, 4,3,"PID");

        //Выгрузил элемент №2
        bot.s1.setPosition(0.4);
        sleep(500);
        bot.s1.setPosition(1);

        //Подъехал к въезду на склад и запросил калибровку координат
        bot.GoTo(0,-6,-90,0.6,3, 3,1,"PID");
        bot.GoTo(0,-6,-90,0.3,1, 1,1,"PID");
        bot.obnul(0.02, -31, -6);

        //Заехал на склад
        bot.GoTo(-45,-12,-90,0.6,10, 3,1,"PID");

        //Взял элемент №3
        bot.GoTo(-65,-9,-90,0.4,10, 4,1,"element");

        //Подъехал к выезду со склада
        bot.GoTo(-40,-9,-90,0.6,3,3,1,"PID");

        //Выехал со склада
        bot.GoTo(0,-6,-90,0.5,3, 3,1,"PID");

        //Преднаклон корзинки
        bot.s1.setPosition(0.8);

        //Подъехал к хабу
        bot.GoTo(15, 40,-43,0.6,2, 4,3,"PID");

        //Выгрузил элемент №3
        bot.s1.setPosition(0.4);
        sleep(500);
        bot.s1.setPosition(1);

        //Подъехал к въезду на склад
        bot.GoTo(0,-6,-90,0.6,5, 3,1,"PID");
        bot.GoTo(0,-6,-90,0.3,2, 1,1,"PID");

        //Заехал на склад
        bot.GoTo(-45,-9,-90,0.6,10, 3,1,"PID");

        //Взял элемент
        bot.GoTo(-65,-9,-90,0.4,10, 5,1,"element");

        //Запись угла в файл
        bot.writeAngle();
    }
}