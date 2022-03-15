package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

//@Disabled
public class Detector extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat(); //Объявляем матрицу
    public enum Location {
        LEFT,
        CENTER,
        RIGHT,
        NOT_FOUND
    }
    private static Location location;

    public Detector(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {

        //Рамки
        final Rect r1 = new Rect(
                new Point(input.cols()*45f/330f, input.rows()*130f/330f),
                new Point(input.cols()*95f/330f, input.rows()*180f/330f));
        final Rect r2 = new Rect(
                new Point(input.cols()*210f/330f, input.rows()*130f/330f),
                new Point(input.cols()*260f/330f, input.rows()*180f/330f));

        //Вспомогательные рамки
        final Rect rv1 = new Rect(
                new Point(input.cols()*75f/330f, input.rows()*125f/330f),
                new Point(input.cols()*125f/330f, input.rows()*175f/330f));
        final Rect rv2 = new Rect(
                new Point(input.cols()*240f/330f, input.rows()*125f/330f),
                new Point(input.cols()*290f/330f, input.rows()*175f/330f));

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2YCrCb); //RGB в YCrCb
        Core.extractChannel(mat, mat, 2); //Синий цвет

        //Чем меньше синего - тем лучше
        //Больше thresh - менее строгое определение
        Imgproc.threshold(mat, mat, 102, 255, Imgproc.THRESH_BINARY_INV);

        Mat mr1 = mat.submat(r1);
        Mat mr2 = mat.submat(r2);
        Mat mrv1 = mat.submat(rv1);
        Mat mrv2 = mat.submat(rv2);

        double centerValue = Core.sumElems(mr1).val[0] / r1.area() / 255;
        double rightValue = Core.sumElems(mr2).val[0] / r2.area() / 255;
        double centerVspomValue = Core.sumElems(mrv1).val[0] / rv1.area() / 255;
        double rightVspomValue = Core.sumElems(mrv2).val[0] / rv2.area() / 255;

        mr1.release();
        mr2.release();
        mrv1.release();
        mrv2.release();

        //Процент нужного цвета в рамке по центру
        telemetry.addData("Center percentage", Math.round(centerValue * 100) + "%");
        //Процент нужного цвета в рамке справа
        telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");

        Scalar white = new Scalar(255, 255, 255);
        Scalar blue = new Scalar (255, 153, 51);
        Scalar red = new Scalar (0, 51, 255);

        //Рисуем рамки
        Imgproc.rectangle(mat, r1, white, 2);
        Imgproc.rectangle(mat, r2, white, 2);

        //Рисуем вспомогательные рамки
        //Imgproc.rectangle(mat, rv1, white, 2);
        //Imgproc.rectangle(mat, rv2, white, 2);

        //Определяем локацию
        //В локации не менее 10% нужного цвета
        if ((centerValue > rightValue) && (Math.round(centerValue * 100) > 9)) {
            location = Location.CENTER;
            telemetry.addLine("Location: CENTER");
        }
        else if ((rightValue > centerValue) && (Math.round(rightValue * 100) > 9)) {
            location = Location.RIGHT;
            telemetry.addLine("Location: RIGHT");
        }
        else if ((Math.round(centerValue * 100) < 9) && (Math.round(rightValue * 100) < 9)) {
            location = Location.LEFT;
            telemetry.addLine("Location: LEFT");
        } else {
            location = Location.NOT_FOUND;
            telemetry.addLine("Location: NOT_FOUND");
        }

        telemetry.update();

        return mat;
    }

    //Метод получения локации
    public static Location getLocation() {
        return (Location) location;
    }
}
