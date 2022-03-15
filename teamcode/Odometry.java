package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import java.lang.Math;

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

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;
import java.io.File;

@Autonomous(name="XYZ", group="XYZ")

public class Odometry extends LinearOpMode {

    public Runnable OdometryThread, MainThread;
    private ElapsedTime runtime = new ElapsedTime();
    public Servo s1;
    public DcMotor m1, m2, m3, m4, m5, m6, m7;
    public DistanceSensor r1;
    NormalizedColorSensor colorLeft, colorRight;
    public BNO055IMU imu;
    public Orientation angles;
    public int strelaold = 1;
    public double zm5, zm6;
    public double x, y;
    public int ur = 800; //Высота одного уровня
    public int baza;
    public double brightLeft, brightRight, threshc;

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

        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        m3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        m4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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


    @Override
    public void runOpMode() throws InterruptedException {

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

                        zm5 = (r1.getDistance(DistanceUnit.MM) > 70) ? 1 : -1;

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

        //Инициализация
        initC();

        waitForStart();

        //Запуск процесса с вычислениями
        OdometryThread O1 = new OdometryThread();
        O1.start();

        while (!isStopRequested() & opModeIsActive()) {}
    }
}