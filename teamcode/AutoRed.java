package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name="AutoRed", group="AutoRed")
//@Disabled
public class AutoRed extends LinearOpMode {

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
            bot.GoTo(-10,30,0,0.35,2,4,1,"PID");
            bot.markerPick();
            bot.s1.setPosition(0.8);
            sleep(500);
        }
        if (bot.baza == 2) {
            bot.GoTo(4,30,0,0.35,2,4,1,"PID");
            bot.markerPick();
            bot.s1.setPosition(0.8);
            sleep(300);
        }
        if (bot.baza == 3) {
            bot.GoTo(6,35,-60,0.2,2,4,1,"PID");
            bot.markerPick();
            bot.s1.setPosition(0.8);
            bot.GoTo(0,37,45,0.35,2,2,1,"PID");
        }

        //Приехал к хабу
        bot.GoTo(-15, 42,43,0.65,2, 2, bot.baza,"PID");

        //Сбросил элемент №1
        bot.s1.setPosition(0.4);
        sleep(500);
        bot.s1.setPosition(1);

        //Подъехал к въезду в склад и запросил калибровку координат
        bot.GoTo(0,-6,90,0.65,5, 4,1,"PID");
        bot.GoTo(0,-6,90,0.3,2, 1,1,"PID");
        bot.obnul(0.02, 31, -6); // обнуление координат

        //Заехал в склад
        bot.GoTo(45,-9,90,0.65,10, 3,1,"PID");

        //Взял элемент №2
        bot.GoTo(65,-8,90,0.5,10, 5,1,"element");

        //Подъехал к выезду со склада и запросил калибровку координат
        bot.GoTo(40,-9,90,0.65,5,2,1,"PID");
        bot.obnul(0.02, 31, -6); // обнуление координат

        //Выехал со склада
        bot.GoTo(0,-9,90,0.5,5, 2,1,"PID");

        //Преднаклон корзины
        bot.s1.setPosition(0.8);

        //Подъехал к хабу
        bot.GoTo(-15, 42,43,0.65,2, 4,3,"PID");

        //Выгрузил элемент №2
        bot.s1.setPosition(0.4);
        sleep(500);
        bot.s1.setPosition(1);

        //Подъехал к въезду на склад и запросил калибровку координат
        bot.GoTo(0,-6,90,0.65,5, 3,1,"PID");
        bot.GoTo(0,-6,90,0.3,2, 1,1,"PID");
        bot.obnul(0.02, 31, -6);

        //Заехал на склад
        bot.GoTo(45,-9,90,0.65,10, 3,1,"PID");

        //Взял элемент №3
        bot.GoTo(65,-9,90,0.4,10, 5,1,"element");

        //Подъехал к выезду со склада и запросил обнуление координат
        bot.GoTo(40,-9,90,0.65,5,2,1,"PID");
        bot.obnul(0.04, 31, -6);

        //Выехал со склада
        bot.GoTo(0,-6,90,0.55,5, 2,1,"PID");

        //Преднаклон корзинки
        bot.s1.setPosition(0.8);

        //Подъехал к хабу
        bot.GoTo(-15, 42,43,0.65,2, 4,3,"PID");

        //Выгрузил элемент №3
        bot.s1.setPosition(0.4);
        sleep(500);
        bot.s1.setPosition(1);

        //Подъехал к въезду на склад и запрсил калибровку координат
        bot.GoTo(0,-6,90,0.55,5, 3,1,"PID");
        bot.obnul(0.02, 31, -6);

        //Заехал на склад
        bot.GoTo(45,-9,90,0.65,10, 3,1,"PID");

        //Взял элемент
        bot.GoTo(65,-9,90,0.4,10, 5,1,"element");

        //Запись угла в файл
        bot.writeAngle();
    }
}