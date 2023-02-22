package org.firstinspires.ftc.teamcode.archived;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

@Disabled
public class TriangleDetector extends OpenCvPipeline {
    enum ConeLocation {
        LEFT,
        RIGHT,
        CENTER,
        NONE
    }
    private Random rng = new Random(12345);
    private int width; // width of the image
    int objectHeightCenter = 0;
    int objectWidthCenter = 0;
    int leftX = 0;
    int rightX = 0;
    ConeLocation location;
    double conArea = 0;
    Mat mat = new Mat();
    Mat temp = new Mat();
    Mat hierarchy = new Mat();
    Mat thresh = new Mat();
    int num;
    int count = 0;


    //Rect leftR = new Rect(new Point(0, 0), new Point(250, 200));
    //Rect centerR = new Rect(new Point(250, 0), new Point(500, 200));
    //Rect rightR = new Rect(new Point(500, 0), new Point(750, 200));

    //Mat left;
    //Mat center;
    //Mat right;

    /**
     *
     * @param width The width of the image (check your camera)
     */
    public TriangleDetector(int width) {
        this.width = width;
    }

    @Override
    public Mat processFrame(Mat input) {
        // "Mat" stands for matrix, which is basically the image that the detector will process
        // the input matrix is the image coming from the camera
        // the function will return a matrix to be drawn on your phone's screen

        // The detector detects regular stones. The camera fits two stones.
        // If it finds one regular stone then the other must be the skystone.
        // If both are regular stones, it returns NONE to tell the robot to keep looking

        // Make a working copy of the input matrix in HSV
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(mat, temp, Imgproc.COLOR_RGB2HSV);

        Imgproc.rectangle(temp, new Point(0, 0), new Point(800, 200), new Scalar(0, 0, 0, 0),50);
        Scalar lowHSV = new Scalar(160, 100, 20); // lower bound HSV for yellow
        Scalar highHSV = new Scalar(179, 255, 255);

        Core.inRange(temp, lowHSV, highHSV, thresh);

        List<MatOfPoint> contours = new ArrayList<>();

        Imgproc.findContours(thresh, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        //Imgproc.drawContours(mat, contours, -1, new Scalar(0,0,255), 4);
        for (int contourIdx=0; contourIdx < contours.size(); contourIdx++ )
        {
            //Minimun size allowed for consideration
            MatOfPoint2f approxCurve = new MatOfPoint2f();
            MatOfPoint2f contour2f = new MatOfPoint2f(contours.get(contourIdx).toArray());

            //Processing on mMOP2f1 which is in type MatOfPoint2f
            double approxDistance = Imgproc.arcLength(contour2f,true)*0.02;
            Imgproc.approxPolyDP(contour2f,approxCurve,approxDistance,true);

            //convert to MatofPoint
            MatOfPoint point = new MatOfPoint(approxCurve.toArray());

            //get boundingrect from contour
            Rect rect = Imgproc.boundingRect(point);
            Imgproc.rectangle(mat,new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), new Scalar(255, 0, 0, 255),3);
            if (Imgproc.contourArea(contours.get(contourIdx)) > conArea && (rect.y + rect.height) > 300) {
                conArea = Imgproc.contourArea(contours.get(contourIdx));
                objectHeightCenter = rect.y + (rect.height/2);
                objectWidthCenter = rect.x + (rect.width/2);
                leftX = rect.x;
                rightX = rect.x + rect.width;
            }
            //bisa Imgproc.rectangle(mRgba, rect.tl(), rect.br(), new Scalar(255, 0, 0),1, 8,0);
            count++;

            //show contour kontur
            //if(Imgproc.contourArea(contours.get(contourIdx))>100) {
            //    Imgproc.drawContours(mat, contours, contourIdx, new Scalar(0,255,0), 5);
            //}
        }
        //Rect r = Imgproc.boundingRect();
        //Imgproc.rectangle(mat, r, new Scalar(0, 255, 255), 4);

        num = contours.size();
        //for (int i = 0; i < contours.size(); i++) {
        //  Moments m = Imgproc.moments(mat);
        //if (m != 0) {

        //}
        //}
        if (objectWidthCenter < 335) {
            location = ConeLocation.LEFT;
        }
        else if (objectWidthCenter < 440 && objectWidthCenter > 335) {
            location = ConeLocation.CENTER;
        }
        else if (objectWidthCenter >= 440){
            location = ConeLocation.RIGHT;
        }
        else {
            location = ConeLocation.NONE;
        }
        //left.release();
        //center.release();
        //right.release();
        //mat.release();

        // if something is wrong, we assume there's no skystone
        //if (mat.empty()) {
        //    location = ConeLocation.NONE;
        //    return input;
        //}
//
//        // We create a HSV range for yellow to detect regular stones
//        // NOTE: In OpenCV's implementation,
//        // Hue values are half the real value
//        Mat thresh = new Mat();
//
//        // We'll get a black and white image. The white regions represent the regular stones.
//        // inRange(): thresh[i][j] = {255,255,255} if mat[i][i] is within the range
//        Core.inRange(mat, lowHSV, highHSV, thresh);
//
//        // Use Canny Edge Detection to find edges
//        // you might have to tune the thresholds for hysteresis
//        Mat edges = new Mat();

//
//        // Iterate and check whether the bounding boxes
//        // cover left and/or right side of the image
//        double left_x = 0.25 * width;
//        double right_x = 0.75 * width;
//        boolean left = false; // true if regular stone found on the left side
//        boolean right = false; // "" "" on the right side
//        for (int i = 0; i != boundRect.length; i++) {
//            if (boundRect[i].x < left_x)
//                left = true;
//            if (boundRect[i].x + boundRect[i].width > right_x)
//                right = true;
//
//
//            // draw red bounding rectangles on mat
//            // the mat has been converted to HSV so we need to use HSV as well
//            Imgproc.rectangle(mat, boundRect[i], new Scalar(0.5, 76.9, 89.8));
//            objectHeight = boundRect[i].y;
//            objectWidth = boundRect[i].x;
//        }
//
//        // if there is no yellow regions on a side
//        // that side should be a Skystone
//        if (!left) location = ConeLocation.LEFT;
//        else if (!right) location = ConeLocation.RIGHT;
//            // if both are true, then there's no Skystone in front.
//            // since our team's camera can only detect two at a time
//            // we will need to scan the next 2 stones
//        else location = ConeLocation.NONE;
//
         temp.release();
         hierarchy.release();
         thresh.release();
        return mat; // return the mat with rectangles drawn
    }

    public ConeLocation getLocation() {
        return this.location;
    }
    public int getHeight() {
        return this.objectHeightCenter;
    }

    public int getWidth() {
        return this.objectWidthCenter;
    }
    public int getNum() {
        return this.num;
    }
    public int getCount() {
        return this.count;
    }
    public int getLeftX() {
        return this.leftX;
    }
    public int getRightX() {
        return this.rightX;
    }
    public int getObjectWidthCenter() { return this.objectWidthCenter;}
}