package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

@Config
public class ConeDetectionPipeline extends MyOpenCvPipeline {

    boolean stopped = false;
    static final int CR_CHAN_HIGH_THRESHOLD = 140;
    static final int CR_CHAN_LOW_THRESHOLD = 10;

    // debug support
    static public boolean debug = true;
    static public int debug_numberOfViews = 8;   // Up to 8
    static public int debug_interval = 6;

    // Color and line thickness for drawing
    static final Scalar TEAL = new Scalar(3, 148, 252);
    static final Scalar PURPLE = new Scalar(158, 52, 235);
    static final Scalar GREEN = new Scalar(0, 255, 0);
    static final int CONTOUR_LINE_THICKNESS = 6;

    // Rect to read avg color (that helps to confirm threshold)
    Rect rectToSample = null;
    static final int rectToSampleSize = 40;
    static public int rectToSampleAdjustX = 0;
    static public int rectToSampleAdjustY = 0;

    // Our working image buffers
    Mat sampleMat = new Mat();
    Mat hlsMat = new Mat();
    Mat hueMat = new Mat();
    Mat hueThresholdMat = new Mat();
    Mat saturationMat = new Mat();
    Mat saturationThresholdMat = new Mat();
    Mat lightMat = new Mat();
    Mat thresholdMat = new Mat();
    Mat morphedThreshold = new Mat();
    Mat ignoreMat = null;
    Mat ignoredMat = new Mat();

    // Result
    static public int sampleColor = -1;
    static public Point conePosition = new Point(0, 0); // center of screen as (0,0)
    static public Point coneCenter = new Point(0, 0);    // upper left corner as (0,0)

     // The elements we use for noise reduction
    Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
    Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(6, 6));

    void morphMask(Mat input, Mat output)
    {
        // Apply some erosion and dilation for noise reduction
        Imgproc.erode(input, output, erodeElement);
        Imgproc.erode(output, output, erodeElement);

        Imgproc.dilate(output, output, dilateElement);
        Imgproc.dilate(output, output, dilateElement);
    }

    ArrayList<MatOfPoint> findContours(Mat input)
    {
        // A list we'll be using to store the contours we find
        ArrayList<MatOfPoint> contoursList = new ArrayList<>();

        // Convert the input image to YCrCb color space, then extract the Cr channel
        Imgproc.cvtColor(input, hlsMat, Imgproc.COLOR_RGB2HLS);
        Core.extractChannel(hlsMat, hueMat, 0); // hue channel
        Core.extractChannel(hlsMat, saturationMat, 2); // saturation channel
        Core.extractChannel(hlsMat, lightMat, 1); // saturation channel

        // Hue for red can be either very low or very high: hue: 170 or 5, light: 141, saturation: 122-164
        // Use saturationThresholdMat to temporarily hold intermediate value
        Imgproc.threshold(hueMat, hueThresholdMat, CR_CHAN_LOW_THRESHOLD, 255, Imgproc.THRESH_BINARY_INV);
        Imgproc.threshold(hueMat, saturationThresholdMat, CR_CHAN_HIGH_THRESHOLD, 255, Imgproc.THRESH_BINARY);
        Core.bitwise_or(hueThresholdMat, saturationThresholdMat, hueThresholdMat);

        // Only keep high saturation pixels, to tell red apart from brown (for example, ~140, light: 84, saturation: 15-60)
        Imgproc.threshold(saturationMat, saturationThresholdMat, 100, 180, Imgproc.THRESH_BINARY);
        Core.bitwise_and(hueThresholdMat, saturationThresholdMat, thresholdMat);

        // Ignore pixels that are too bright (which is almost white but has a light red hue)
        Imgproc.threshold(lightMat, lightMat, 0, 150, Imgproc.THRESH_BINARY);
        Core.bitwise_and(thresholdMat, lightMat, thresholdMat);

        morphMask(thresholdMat, morphedThreshold);

        // Ok, now actually look for the contours! We only look for external contours.
        Imgproc.findContours(morphedThreshold, contoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        Log.v("pipeline", String.format("Found %d contours.", contoursList.size()));
        return contoursList;
    }

    void drawRotatedRect(RotatedRect rect, Mat input, boolean halfThickness) {
        // Draws a rotated rect by drawing each of the 4 lines individually
        Point[] points = new Point[4];
        rect.points(points);

        for (int i = 0; i < 4; ++i) {
            int thickness = CONTOUR_LINE_THICKNESS;
            if (halfThickness) {
                thickness /= 2;
            }
            Imgproc.line(input, points[i], points[(i + 1) % 4], GREEN, thickness);
        }
    }

    void updateIgnoreMask(RotatedRect rect) {

        Rect boundingRect = rect.boundingRect();
        Rect upperMost = new Rect(boundingRect.x, boundingRect.y, boundingRect.width, boundingRect.height * 3/4);
        Imgproc.rectangle(ignoreMat, upperMost, new Scalar(255), -1);
    }

    RotatedRect rectFitToContour(MatOfPoint contour, boolean filter) {
        // Transform the contour to a different format
        Point[] points = contour.toArray();
        MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());

        // Do a rect fit to the contour
        RotatedRect rotatedRectFitToContour = Imgproc.minAreaRect(contour2f);

        Log.v("pipeline", String.format("Contour size: %4.2f", rotatedRectFitToContour.size.area()));

        if (filter) {
            if (rotatedRectFitToContour.size.area() < 1000) {
                return null;
            }

//            Rect boundingRect = rotatedRectFitToContour.boundingRect();
//            if (boundingRect.height / boundingRect.width < 1.2) {
//                return false;
//            }
        }

        return rotatedRectFitToContour;
    }

    ArrayList<Mat> inputMatToHLS(Mat input) {

        Imgproc.cvtColor(input, sampleMat, Imgproc.COLOR_RGB2HLS);
        ArrayList<Mat> HLSChannels = new ArrayList<Mat>(3);
        Core.split(sampleMat, HLSChannels);
        return HLSChannels;
    }

    int calculateSampleColor(Mat input) {

        // Calculate the avg color of a small squre at center of screen, used to confirm threadthold, etc.
        int centerX = input.width()/2;
        int centerY = input.height()/2;
        int adjustX = - rectToSampleSize/2 + rectToSampleAdjustX;
        int adjustY = - rectToSampleSize/2 + rectToSampleAdjustY;
        rectToSample = new Rect(centerX + adjustX, centerY + adjustY, rectToSampleSize, rectToSampleSize);

        Mat colorAvgArea = input.submat(rectToSample);
        ArrayList<Mat> matInHLS = inputMatToHLS(colorAvgArea);
        int avgH = (int) Core.mean(matInHLS.get(0)).val[0];
        int avgL = (int) Core.mean(matInHLS.get(1)).val[0];
        int avgS = (int) Core.mean(matInHLS.get(2)).val[0];
        colorAvgArea.release(); // don't leak memory!
        matInHLS.get(0).release(); // don't leak memory!
        matInHLS.get(1).release(); // don't leak memory!
        matInHLS.get(2).release(); // don't leak memory!

        Log.v("sample", String.format("Average hue/light/saturation of sample rect (in teal): %d, %d, %d",
                avgH, avgL, avgS));
        return avgH;
    }

    @Override
    public Mat processFrame(Mat input) {
        if (stopped) {
            return input;
        }

        sampleColor = calculateSampleColor(input);

        Log.v("pipeline", "processFrame starting.");
        long startedAt = System.currentTimeMillis();

        ArrayList<MatOfPoint> contours = findContours(input);
        int index = 0;
        int indexForCone = 0;

        // Initialize the ignoreMat(mask) to ignore everything. This will be used to refine contours found
        // in the 1st round.
        // Do it here (instead of earlier) to ensure it is same size/type as morphedThreshold mat.
        if (ignoreMat == null) {
            ignoreMat = new Mat(morphedThreshold.size(), morphedThreshold.type());
        }
        Imgproc.rectangle(ignoreMat, new Rect(new Point(0, 0), ignoreMat.size()), new Scalar(0), -1);

        // First round finding contours, and rect that fits contours.
        for (index = 0; index < contours.size(); index ++) {
            Log.v("pipeline", String.format("Checking contour %d(/%d)", index, contours.size()));
            RotatedRect rectFits = rectFitToContour(contours.get(index), true);
            if (rectFits != null) {
                drawRotatedRect(rectFits, input, true);
                updateIgnoreMask(rectFits);
            }
        }

        // The bottom part of the cone often has shade and thus has more noise. Sometimes the rect fits
        // the contour is not upright. Create a mask that only contains the upper 3/4 of cones, and find
        // contours for the 2nd round, hopefully to get a more accurate cone position.
        Core.bitwise_and(morphedThreshold, ignoreMat, ignoredMat);
        ArrayList<MatOfPoint> contoursList = new ArrayList<>();
        Imgproc.findContours(ignoredMat, contoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
        contours = contoursList;

        for (index = 0; index < contours.size(); index ++ ) {
            RotatedRect rectFits = rectFitToContour(contours.get(index), true);
            if (rectFits != null) {
                drawRotatedRect(rectFits, input, false);
                coneCenter = rectFits.center;
            }
        }

        // Only draw the contour for the cone. (Pass -1 to draw all contours)
        Imgproc.drawContours(input, contours, -1, PURPLE, CONTOUR_LINE_THICKNESS);
        Imgproc.drawMarker(input, coneCenter, GREEN, Imgproc.MARKER_STAR, 20, 3);
        Imgproc.rectangle(input, rectToSample, TEAL, CONTOUR_LINE_THICKNESS);

        Log.v("pipeline", "processFrame returning.");

        if (debug) {
            TelemetryPacket packet = new TelemetryPacket();
            long now = System.currentTimeMillis();
            packet.put("Time spent (ms):", now - startedAt);
            int outputMode = ((int) (now / (debug_interval * 1000))) % debug_numberOfViews;
            Mat matToReturn = null;
            switch (outputMode) {
                case 0:
                    packet.put("current view", "input");
                    matToReturn = input;
                    break;
                case 1:
                    packet.put("current view", "hue");
                    matToReturn = hueMat;
                    break;
                case 2:
                    packet.put("current view", "hue within threshold");
                    matToReturn = hueThresholdMat;
                    break;
                case 3:
                    packet.put("current view", "saturation");
                    matToReturn = saturationMat;
                    break;
                case 4:
                    packet.put("current view", "saturation within threshold");
                    matToReturn = saturationThresholdMat;
                    break;
                case 5:
                    packet.put("current view", "within threshold (combined)");
                    matToReturn = thresholdMat;
                    break;
                case 6:
                    packet.put("current view", "morphed");
                    matToReturn = morphedThreshold;
                    break;
                case 7:
                    packet.put("current view", "for refinement");
                    matToReturn = ignoredMat;
                    break;
                default:
                    packet.put("current view", "input");
                    matToReturn = input;
                    break;
            }
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            return matToReturn;
        } else {
            return input;
        }
    }

    public void stop() {
        stopped = true;
    }
}
