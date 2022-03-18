package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import java.io.File;

@TeleOp(name="TelepopBlue", group="Tesla")
//@Disabled
public class TelepopBlue extends LinearOpMode {
    private DcMotor m1, m2, m3, m4, m5, m6, m7, led;
    public DistanceSensor r1;
    private double a, b, vyr, e1, e2, prirost, turn, zm1, zm2, zm3, zm4, zm7, zm5, zm6, zs1, zs3, zs4, zLED;
    Servo s1, s2, s3, s4;
    private BNO055IMU imu;
    private Orientation angles;
    public NormalizedColorSensor colorLeft, colorRight;
    public int strelaold = 1;
    public String laststyle = "stop";
    public double x, y, obx, oby;
    public int ur = 800; //Высота одного уровня
    public float gain = 2; //Коэффициент датчиков цвета
    public double brightLeft, brightRight, started, pow;
    public int baza = 3;
    public boolean obnulit = false;
    double distance_mm;
    private float dgr = 0;
    double LastAngle = 0;
    double OldAngle, strelaPos;
    int prevx, prevy;
    public double volt = 0;
    public boolean reinit = false;
    public boolean reverse = false;
    public boolean auto = false;
    private ElapsedTime runtime = new ElapsedTime();
    public double threshc = 0.02;

    File xFile = AppUtil.getInstance().getSettingsFile("x.txt"); //Файл с координатой x
    File yFile = AppUtil.getInstance().getSettingsFile("y.txt"); //Файл с координатой y

    File angleFile = AppUtil.getInstance().getSettingsFile("angle.txt"); //Файл с отклонением угла в конце автономки от начального угла
    File strelaFile = AppUtil.getInstance().getSettingsFile("strela.txt"); //Файл с позицией стрелы

    public void initIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void initC() {
        //Ищем датчики и моторы в конфигурации
        m1 = hardwareMap.get(DcMotor.class, "m1");
        m2 = hardwareMap.get(DcMotor.class, "m2");
        m3 = hardwareMap.get(DcMotor.class, "m3");
        m4 = hardwareMap.get(DcMotor.class, "m4");
        m5 = hardwareMap.get(DcMotor.class, "m5");
        m6 = hardwareMap.get(DcMotor.class, "m6");
        m7 = hardwareMap.get(DcMotor.class, "m7");
        led = hardwareMap.get(DcMotor.class, "led");
        s1 = hardwareMap.get(Servo.class, "s1");
        s3 = hardwareMap.get(Servo.class, "s3");
        s4 = hardwareMap.get(Servo.class, "s4");
        r1 = hardwareMap.get(DistanceSensor.class, "r1");
        initIMU();

        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m6.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m7.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Обнуляем энкодеры моторов
        m7.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        m7.setDirection(DcMotorSimple.Direction.REVERSE);

        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m6.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m6.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        r1 = hardwareMap.get(DistanceSensor.class, "r1");

        int relativeLayoutIdLeft = hardwareMap.appContext.getResources().getIdentifier("color.left", "id", hardwareMap.appContext.getPackageName());
        colorLeft = hardwareMap.get(NormalizedColorSensor.class, "color.left");

        int relativeLayoutIdRight = hardwareMap.appContext.getResources().getIdentifier("color.right", "id", hardwareMap.appContext.getPackageName());
        colorRight = hardwareMap.get(NormalizedColorSensor.class, "color.right");

    }

    @Override
    public void runOpMode() {

        class CalcThread implements Runnable {
            private Thread t;
            private boolean running;

            public void run() {
                telemetry.addLine("Calc thread running");
                telemetry.update();

                try {

                    m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    m3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    m4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    prevx = Integer.parseInt(ReadWriteFile.readFile(xFile).trim());
                    prevy = Integer.parseInt(ReadWriteFile.readFile(yFile).trim());

                    LastAngle = Double.parseDouble(ReadWriteFile.readFile(angleFile).trim()); //Отклонение угла от начального
                    OldAngle = LastAngle;

                    strelaPos = Double.parseDouble(ReadWriteFile.readFile(strelaFile).trim()); //Позиция стрелы в начале телеопа

                    while (!isStopRequested() & opModeIsActive()) {

                        //gamepad2.a - повернуть корзинку
                        //gamepad2.left_stick_y - сборщик
                        //gamepad2.right_stick_x - барабан для уток
                        //gamepad2.right_stick_y - стрела
                        //gamepad2.dpad_down - опускание стрелы
                        //gamepad2.dpad_left - блокировка стрелы
                        //gamepad2.dpad_up - блокировка барабана

                        LastAngle = !reinit ? LastAngle : 0;

                        a = (gamepad1.left_bumper ? 1 : 0.6) - (gamepad1.right_bumper ? 0.3 : 0); //Коэффициент скорости

                        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                        vyr = ((angles.firstAngle + LastAngle + 90) / 100) * (gamepad1.a ? 1 : 0); //Выравнивание
                        turn = (gamepad1.right_stick_x * a) * (gamepad1.a ? 0 : 1); //Поворот

                        zm1 = (-gamepad1.left_stick_y - gamepad1.left_stick_x) * a - vyr - turn;
                        zm2 = (-gamepad1.left_stick_y + gamepad1.left_stick_x) * a - vyr - turn;
                        zm3 = (gamepad1.left_stick_y + gamepad1.left_stick_x) * a - vyr - turn;
                        zm4 = (gamepad1.left_stick_y - gamepad1.left_stick_x) * a - vyr - turn;

                        zm5 = gamepad2.left_stick_y * (gamepad2.left_stick_y > 0 ? 0.4 : 1); //Сборщик
                        if (gamepad2.right_trigger > 0.15 & gamepad2.left_stick_y < 0) {zm5 = -1;}
                        zm6 = gamepad2.right_stick_y * 0.9 * (gamepad2.dpad_left ? 0 : 1); //Стрела

                        //Опускание стрелы
                        if (gamepad2.dpad_down) {
                            if (m6.getCurrentPosition() > 10 - strelaPos) {
                                zm6 = 0.5;
                            }
                            if (m6.getCurrentPosition() < 10 - strelaPos) {
                                zm6 = -0.5;
                            }
                        }

                        m6.setPower(zm6);

                        if (gamepad2.b) {
                            zm7 = -1;
                        }
                        else if (gamepad2.x) {
                            zm7 = -0.55;
                        } else {
                            zm7 = 0;
                        }
                        zm7 = zm7 * (gamepad2.dpad_up ? 0 : 1); //Уточный барабан

                        zs1 = gamepad2.a ? 0.4 : 1; //Поворот корзинки

                        //Опустить руку
                        zs4 = gamepad2.left_bumper ?
                                0.1 + ((gamepad2.left_trigger > 0.15) ? 0.3 : 0) :
                                1 - ((gamepad2.left_trigger > 0.15) ? 0.6 : 0);

                        zs3 = (gamepad2.y ? 1 : 0.6); //Отпустить маркер

                        if (zm5 != 0 || zs4 != 1) {
                            zs1 = 1;
                        } else if (gamepad2.a) {
                            zs1 = 0.4;
                        } else {
                            zs1 = 0.9;
                        }


                        if (r1.getDistance(DistanceUnit.MM) > 70) {
                            zLED = Math.abs(Math.sin(runtime.seconds())/2 + 0.5);
                        } else {
                            zLED = Math.round(runtime.milliseconds() / 200) % 2 == 0 ? 1 : 0;
                        }

                    }

                } catch (Exception e) {
                    telemetry.addLine("Calc thread interrupted");
                    telemetry.update();

                }
            }

            public void start () {
                if (t == null) {
                    t = new Thread (this, "Calc thread");
                    t.start();
                }
            }
        }

        //Инициализация
        initC();

        waitForStart();

        //Запуск процесса с вычислениями
        CalcThread C1 = new CalcThread();
        C1.start();

        //ОСНОВНАЯ ПРОГРАММА

        while(opModeIsActive() & !isStopRequested()) {

            led.setPower(zLED);

            m1.setPower(zm1);
            m2.setPower(zm2);
            m3.setPower(zm3);
            m4.setPower(zm4);
            m5.setPower(zm5);
            m7.setPower(zm7);

            s1.setPosition(zs1);
            s3.setPosition(zs3);
            s4.setPosition(zs4);

            if (gamepad1.y) { initIMU(); reinit = true; } //Обнуление гироскопа

            while (gamepad1.b) { //Заезд на склад

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                vyr = (angles.firstAngle + LastAngle + 90) / 100;

                m1.setPower(-1 - vyr);
                m2.setPower(-0.7 - vyr);
                m3.setPower(1 - vyr);
                m4.setPower(0.7 - vyr);

                telemetry.addData("vyr", vyr);
                telemetry.update();
            }

            while (gamepad1.x) { //Выезд со склада

                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                vyr = (angles.firstAngle + LastAngle + 90) / 100;

                m1.setPower(0.65 - vyr);
                m2.setPower(1 - vyr);
                m3.setPower(-0.65 - vyr);
                m4.setPower(-1 - vyr);

                telemetry.addData("vyr", vyr);
                telemetry.update();
            }

            telemetry.addData("prevX", prevx);
            telemetry.addData("prevY", prevy);
            telemetry.addData("Угол начала телеопа", OldAngle);
            telemetry.addData("Позиция стрелы в начале", strelaPos);
            telemetry.addData("Стрела", m6.getCurrentPosition());
            telemetry.update();

        }

        ReadWriteFile.writeFile(angleFile, String.valueOf(0));
        ReadWriteFile.writeFile(strelaFile, String.valueOf(0));

        ReadWriteFile.writeFile(xFile, String.valueOf(0));
        ReadWriteFile.writeFile(yFile, String.valueOf(0));

    }
}