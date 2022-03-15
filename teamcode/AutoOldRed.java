package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.io.File;

@Autonomous(name="AutoOldRed", group="Auto")
@Disabled
public class AutoOldRed extends LinearOpMode {

    public Runnable OdometryThread, MainThread;
    public Servo s1, s2;
    public DcMotor m1, m2, m3, m4, m5, m6, m7;
    public DistanceSensor r1;
    public BNO055IMU imu;
    NormalizedColorSensor colorLeft, colorRight;
    public Orientation angles;
    public int strelaold = 1;
    public String laststyle = "stop";
    public double zm5, zm6;
    public double x, y, obx, oby;
    public int ur = 800; //Высота одного уровня
    float gain = 2; //Коэффициент датчиков цвета
    public double brightLeft, brightRight, threshc;
    public int baza;
    public boolean obnulit = false;
    private ElapsedTime runtime = new ElapsedTime();

    File angleFile = AppUtil.getInstance().getSettingsFile("angle.txt");

    public void initIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void initC() {
        initIMU();

        m1 = hardwareMap.get(DcMotor.class, "m1");
        m2 = hardwareMap.get(DcMotor.class, "m2");
        m3 = hardwareMap.get(DcMotor.class, "m3");
        m4 = hardwareMap.get(DcMotor.class, "m4");
        m5 = hardwareMap.get(DcMotor.class, "m5");
        m6 = hardwareMap.get(DcMotor.class, "m6");
        m7 = hardwareMap.get(DcMotor.class, "m7");

        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m6.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m7.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        m7.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m7.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m5.setDirection(DcMotor.Direction.REVERSE);

        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m6.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        s1 = hardwareMap.get(Servo.class, "s1");
        s1.setPosition(1);

        r1 = hardwareMap.get(DistanceSensor.class, "r1");

        int relativeLayoutIdLeft = hardwareMap.appContext.getResources().getIdentifier("color.left", "id", hardwareMap.appContext.getPackageName());
        colorLeft = hardwareMap.get(NormalizedColorSensor.class, "color.left");

        int relativeLayoutIdRight = hardwareMap.appContext.getResources().getIdentifier("color.right", "id", hardwareMap.appContext.getPackageName());
        colorRight = hardwareMap.get(NormalizedColorSensor.class, "color.right");

    }

    public void obnul(double thresh, double newx, double newy) {
        obnulit = true;
        threshc = thresh;
        obx = newx;
        oby = newy;
    }
    
    public void GoTo(double tox, double toy, float rad, double spd, double timeout, int strela, String style) {

        double dx, dy, gx, gy, zm1, zm2, zm3, zm4, cor, accel, antiaccel;
        float sin, cos, ang, c;
        boolean done = false;
        boolean stdone = false;

        m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m5.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m6.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        runtime.reset();

        rad = (float) Math.toRadians(rad);

        while ((!done | !stdone) & opModeIsActive() & runtime.seconds() < timeout) {

            dx = tox - x;
            dy = toy - y;
            c = (float) Math.sqrt(dx * dx + dy * dy);

            cos = (float) ((tox - x) / c);
            sin = (float) ((toy - y) / c);

            ang = (float) Math.asin(sin);

            if (cos < 0) {
                if (sin >= 0) {
                    ang = (float) (3.14 - ang);
                } else {
                    ang = (float) (-3.14 - ang);
                }
            }

            ang -= angles.firstAngle;

            gy = Math.sin(ang);
            gx = Math.cos(ang);


            accel = (laststyle == "continue") ? 1 : Range.clip(runtime.milliseconds() / 100, 0.8, 1);
            antiaccel = Range.clip(c / 20, 0.2, 0.4);

            zm1 = Range.clip(gy - gx, -1, 1) * accel;
            zm2 = Range.clip(gy + gx, -1, 1) * accel;
            zm3 = Range.clip(-gy + gx, -1, 1) * accel;
            zm4 = Range.clip(-gy - gx, -1, 1) * accel;
            cor = Range.clip(0.4 * (rad - angles.firstAngle), -0.2, 0.2);




            switch (style) {
                case "stop":
                    // обычная езда
                    if (c > 15) {
                        m1.setPower(spd * (zm1 + cor));
                        m2.setPower(spd * (zm2 + cor));
                        m3.setPower(spd * (zm3 + cor));
                        m4.setPower(spd * (zm4 + cor));
                    }

                    // плавный доезд
                    if ((c <= 15) & (c > 1)) {
                        m1.setPower(antiaccel * zm1 + cor);
                        m2.setPower(antiaccel * zm2 + cor);
                        m3.setPower(antiaccel * zm3 + cor);
                        m4.setPower(antiaccel * zm4 + cor);
                    }

                    // докручивание
                    if (c <= 1) {
                        m1.setPower(cor * 2);
                        m2.setPower(cor * 2);
                        m3.setPower(cor * 2);
                        m4.setPower(cor * 2);
                    }

                    // условие остановки
                    if ((c <= 1) & (Math.abs(cor) < 0.07)) {
                        m1.setPower(0);
                        m2.setPower(0);
                        m3.setPower(0);
                        m4.setPower(0);
                        done = true;
                    }
                    break;

                case "continue":
                    // обычная езда
                    if (c > 1) {
                        m1.setPower(spd * (zm1 + cor));
                        m2.setPower(spd * (zm2 + cor));
                        m3.setPower(spd * (zm3 + cor));
                        m4.setPower(spd * (zm4 + cor));
                    }

                    // условие остановки
                    if (c <= 1) {
                        done = true;
                    }

                    if (laststyle == "element") {
                        m5.setPower(zm5 * (Math.round(runtime.milliseconds() / 200) % 2 == 0 ? 1 : 0));
                    }
                    break;

                case "element":
                    m5.setPower(zm5 * (Math.round(runtime.milliseconds() / 200) % 2 == 0 ? 1 : 0));
                    // обычная езда
                    if (c > 20) {
                        m1.setPower(spd * (zm1 + cor));
                        m2.setPower(spd * (zm2 + cor));
                        m3.setPower(spd * (zm3 + cor));
                        m4.setPower(spd * (zm4 + cor));
                    }

                    // докручивание
                    if (c <= 2) {
                        m1.setPower(cor * 3);
                        m2.setPower(cor * 3);
                        m3.setPower(cor * 3);
                        m4.setPower(cor * 3);
                    }

                    // условие остановки
                    if (r1.getDistance(DistanceUnit.MM) < 70) {
                        m1.setPower(0);
                        m2.setPower(0);
                        m3.setPower(0);
                        m4.setPower(0);
                        done = true;
                    }
                    break;
            }

            if (!stdone & (zm6 < 0)) {
                if (m6.getCurrentPosition() > ((strela - 1) * ur)) {
                    m6.setPower(0);
                    stdone = true;
                }
            }

            if (!stdone & (zm6 > 0)) {
                if (m6.getCurrentPosition() < ((strela - 1) * ur)) {
                    m6.setPower(0);
                    stdone = true;
                }
            }

            if (obnulit & brightLeft > threshc & brightRight > threshc) {
                x = obx;
                y = oby;
                obnulit = false;
            }
            
            zm6 = (0.5 * (strelaold < strela ? -1 : 1) * (!stdone ? 1 : 0) * (strelaold == strela ? 0 : 1));
            m6.setPower(zm6);
        }

        strelaold = strela;
        laststyle = style;
        if (style == "stop") {
            m1.setPower(0);
            m2.setPower(0);
            m3.setPower(0);
            m4.setPower(0);
        }
        m5.setPower(0);
        m6.setPower(0);
    }

    class OdometryThread implements Runnable {
        private Thread t;
        private boolean running;

        public void run() {
            telemetry.addLine("Odometry thread running");
            telemetry.update();

            try {

                m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                m5.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

                double alpha, theta, r, deltay, deltax;
                int encx, ency, oldx = 0, oldy = 0;

                alpha = angles.firstAngle;

                float k = (float) 336.360;
                double pi = Math.PI;

                x = 0;
                y = 0;

                while (!isStopRequested() & opModeIsActive()) {

                    NormalizedRGBA colorsLeft = colorLeft.getNormalizedColors();
                    NormalizedRGBA colorsRight = colorRight.getNormalizedColors();

                    brightLeft = (colorsLeft.red + colorsLeft.green + colorsLeft.blue) / 3;
                    brightRight = (colorsRight.red + colorsRight.green + colorsRight.blue) / 3;

                    zm5 = (r1.getDistance(DistanceUnit.MM) > 70) ? 1 : -0.7;

                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

                    encx = m2.getCurrentPosition() - oldx;
                    ency = m5.getCurrentPosition() - oldy;

                    theta = angles.firstAngle - alpha;

                    if (theta != 0) {
                        r = ((ency / theta) + k);
                        deltay = (Math.sin(theta) * r);
                        deltax = (Math.cos(theta) * r - r);

                        y += (Math.sin(alpha) * deltax + Math.sin(alpha + pi / 2) * deltay + Math.sin(alpha) * encx) / 80;
                        x += (Math.cos(alpha) * deltax + Math.cos(alpha + pi / 2) * deltay + Math.cos(alpha) * encx) / 80;

                        alpha = angles.firstAngle;
                        oldx = m2.getCurrentPosition();
                        oldy = m5.getCurrentPosition();
                    } else {
                        deltay = (float) (ency * Math.cos(alpha));
                        deltax = (float) (-ency * Math.sin(alpha));

                        y += (deltay + Math.sin(alpha) * encx) / 80;
                        x += (deltax + Math.cos(alpha) * encx) / 80;

                        alpha = angles.firstAngle;
                        oldx = m2.getCurrentPosition();
                        oldy = m5.getCurrentPosition();
                    }
                    
                    telemetry.addData("X", Math.round(x));
                    telemetry.addData("Y", Math.round(y));
                    telemetry.addData("Яркость слева: ", brightLeft);
                    telemetry.addData("Яркость справа: ", brightRight);
                    telemetry.addData("Направление", Math.toDegrees(angles.firstAngle));
                    telemetry.update();
                }

            } catch (Exception e) {
                telemetry.addLine("Odometry thread interrupted");
                telemetry.update();

            }
        }

        public void start() {
            if (t == null) {
                t = new Thread(this, "Odometry thread");
                t.start();
            }
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        //Запуск камеры
        OpenCvCamera webcam;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        webcam.setPipeline(new Detector(telemetry));
        webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

        //Инициализация
        initC();

        while (!opModeIsActive() & !isStopRequested()) {
            telemetry.addData("Distance: ", String.format("%.01f mm", r1.getDistance(DistanceUnit.MM)));
            telemetry.update();
        }

        waitForStart();

        //Запуск одометрии во втором потоке
        OdometryThread O1 = new OdometryThread();
        O1.start();

        //ОСНОВНАЯ ПРОГРАММА
        webcam.stopStreaming();

        switch (Detector.getLocation()) {
            case LEFT:
                baza = 1;
                break;
            case CENTER:
                baza = 2;
                break;
            case RIGHT:
                baza = 3;
                break;
            case NOT_FOUND:
                baza = 3;
                break;
        }


        GoTo(-20, 45, 43, 0.5, 3, baza, "stop"); // порт

        sleep(50);
        s1.setPosition(0.15); // сброс элемента
        sleep(400);
        s1.setPosition(1);

        GoTo(4, -5.5, 90, 0.6, 1.8, 1, "continue"); // склад
        GoTo(65, -5.5, 90, 0.55, 2.75, 1, "element"); // элемент
        GoTo(65, -5, 90, 0.4, 0.5, 1, "stop"); // выравнивание
        obnul(0.04, 31, -6); // обнуление координат
        GoTo(0, -5.5, 90, 0.5, 2.5, 1, "continue"); // склад

        GoTo(-20, 45, 43, 0.5, 3.25, 3, "stop"); // порт

        sleep(50);
        s1.setPosition(0.15); // сброс элемента
        sleep(400);
        s1.setPosition(1);

        GoTo(4, -5.5, 90, 0.6, 1.8, 1, "continue"); // склад
        GoTo(65, -5.5, 90, 0.55, 2.75, 1, "element"); // элемент
        GoTo(65, -5, 90, 0.4, 0.5, 1, "stop"); // выравнивание
        obnul(0.04, 31, -6); // обнуление координат
        GoTo(0, -5.5, 90, 0.5, 2.5, 1, "continue"); // склад

        GoTo(-20, 45, 43, 0.5, 3.25, 3, "stop"); // порт

        sleep(100);
        s1.setPosition(0.15); // сброс элемента
        sleep(400);
        s1.setPosition(1);

        GoTo(3, -5.5, 90, 0.5, 2.5, 1, "continue"); // склад
        GoTo(55, -5.5, 90, 0.5, 2.5, 1, "element"); // парковка

        //Запись угла в файл
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        ReadWriteFile.writeFile(angleFile, String.valueOf(angles.firstAngle));

    }
}