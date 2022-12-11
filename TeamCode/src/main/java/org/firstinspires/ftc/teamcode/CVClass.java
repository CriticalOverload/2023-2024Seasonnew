package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.vuforia.Box3D;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
public class CVClass extends OpenCvPipeline{
    public static double satlow = 0;
    public static double sathi = 255;
    public static double vallow = 80;
    public static double valhi = 0;
    public static double pdiffs = 20;
    private double boxPDiffs = 0.0;
    private List<MatOfPoint> blackContours;
    Mat output = new Mat();//output mat

    Rect blackbox,yellowbox;//draws box

    int signal = 0; //1 left, 2 mid, 3 right. 0 means default to left.

    @Override
    public Mat processFrame(Mat input){
        /*
        step 1: get max yellow
        step 2: if there is yellow, get max black. else return 0
        step 3: if there is no black, then return 2. else return 1.
        */
        //NOTE!!!!!
        //H:[0,179]
        //S:[0,255]
        //V:[0,255]
        //.... thanks opencv
        //https://docs.opencv.org/4.x/df/d9d/tutorial_py_colorspaces.html
        //also test using RETR_TREE idk what the difference is. probably just method todo

        List<MatOfPoint> yellowContours = getGreen(input);
        blackContours = getBlack(input);//may make this black?
        input.copyTo(output);

        //need to draw the rectangles first...

        if(yellowContours.size() > 0){
            double max = 0;
            int maxInd = 0;
            for(int i = 0; i < yellowContours.size(); i++){//loop through all the contours, and find the largest box
                double area = Imgproc.contourArea(yellowContours.get(i));
                if(area > max){
                    max = area;
                    maxInd = i;
                }
            }
            //draw a box of the largest one of single color
            Rect largestRect = Imgproc.boundingRect(yellowContours.get(maxInd));
            yellowbox = largestRect;
            Scalar boxColor = new Scalar(0, 255, 255);//not white... something!!
            Imgproc.rectangle(output, yellowbox, boxColor, 3, 8, 0);//Currently boxed based on rectangle
        }

        if(blackContours.size()> 0){
            double max = 0;
            int maxInd = 0;
            for(int i = 0; i < blackContours.size(); i++){//loop through all the contours, and find the largest box
                double area = Imgproc.contourArea(blackContours.get(i));
                if(area > max){
                    max = area;
                    maxInd = i;
                }
            }
            //draw a box of the largest one of single color
            Rect largestRect = Imgproc.boundingRect(blackContours.get(maxInd));
            blackbox = largestRect;
            Scalar boxColor = new Scalar(255, 255, 255);//box should be white?
            Imgproc.rectangle(output, blackbox, boxColor, 3, 8, 0);//Currently boxed based on rectangle
        }

        if(yellowContours.size() > 0){//in other words, it found yellow
            if(blackContours.size() > 0){//found black... change height!!!
                boxPDiffs = 200.0*Math.abs(blackbox.height-yellowbox.height)/(double)(blackbox.height+ yellowbox.height);//percent difference: |a-b|/((a+b)/2)*100=200*|a-b|/(a+b)//todo use area?

                if (boxPDiffs < pdiffs){
                    //yellow and black
                    //around 123ish
                    signal = 2;
                }
                else{
                    signal = 3;//fix this if later... todo
                }
            }
            else {
                //just yellow
                //sees yellow but doesn't see black
                //sees yellow and black, but box sizes aren't similar
                signal = 3;
            }
        }
        else{
            //no yellow, signal should be 1
            signal = 1;
        }
        return output;
    }

    //unrelated...
    //blog.jcole.us/2017/04/13/wireless-programming-for-ftc-robots
    //related
    //https://alloyui.com/examples/color-picker/hsv.html
    public List<MatOfPoint> getGreen(Mat input){
        Mat output = new Mat();//output mat
        Mat hsv = new Mat();//after hsv
        Mat blur = new Mat();//after blur
        Mat singleColor = new Mat();//after single color
        Mat hierarchy = new Mat();//after ?

        List<MatOfPoint> contours = new ArrayList<>();

        input.copyTo(output);//don't modify inputs


        //step 1: blur
        Imgproc.GaussianBlur(output, blur, new Size(5,5), 0);//source, destination, size of blur, sigmax (not too important) todo

        //step 2: rgb to hsv
        //or bgr???
        Imgproc.cvtColor(blur, hsv, Imgproc.COLOR_RGB2HSV);//source, dest, color swap choice (is an int technically)

        //find contours:
        //using HSV
        double hue = 25;
        double sensitivity = 10;//to adjust for slight variations. Ewww that's a giant sensitivity...

//        Scalar lowBound = new Scalar((hue/2)-sensitivity,100,50);//lower bound... may be wrong???
        Scalar lowBound = new Scalar(hue-sensitivity,100,127);//lower bound
        Scalar hiBound = new Scalar(hue+sensitivity,255,255);//higher bound
        Core.inRange(hsv, lowBound, hiBound, singleColor);//source, low bound, high bound, destination. Gets all color within given range, removes all others
        Imgproc.findContours(singleColor, contours, hierarchy, Imgproc.RETR_LIST,Imgproc.CHAIN_APPROX_SIMPLE);//source, contours list, hierarchy mat, int for code, and int method

        return contours;
    }

    public List<MatOfPoint> getBlack(Mat input){
        Mat output = new Mat();//output mat
        Mat hsv = new Mat();//after hsv
        Mat blur = new Mat();//after blur
        Mat singleColor = new Mat();//after single color
        Mat hierarchy = new Mat();//after ?

        List<MatOfPoint> contours = new ArrayList<>();

        double saturation, value;
        double sensitivity;

        input.copyTo(output);//don't modify inputs

        //using HSV

        //for black, sat and value matter but hue doesn't really.
        saturation = 0;
        value = 255;
        sensitivity = 30;//to adjust for slight variations. Ewww that's a giant sensitivity...

        //step 1: blur
        Imgproc.GaussianBlur(output, blur, new Size(5,5), 0);//source, destination, size of blur, sigmax (not too important) todo

        //step 2: rgb to hsv
        Imgproc.cvtColor(blur, hsv, Imgproc.COLOR_RGB2HSV);//source, dest, color swap choice (is an int technically)

        //find contours:
        Scalar lowBound = new Scalar(0,satlow,vallow);//saturation,value - sensitivity);//lower bound
        Scalar hiBound = new Scalar(179,sathi,valhi);//saturation + sensitivity,value);//higher bound
        Core.inRange(hsv, lowBound, hiBound, singleColor);//source, low bound, high bound, destination. Gets all color within given range, removes all others
        Imgproc.findContours(singleColor, contours, hierarchy, Imgproc.RETR_LIST,Imgproc.CHAIN_APPROX_SIMPLE);//source, contours list, hierarchy mat, int for code, and int method
        return contours;
    }

    public int getSignal() {
        return signal;
    }

    public int getHeight() {
        if (blackbox != null)
            return blackbox.height;
        return -1;
    }
    public double getBoxPDiffs(){return boxPDiffs;}

    public int getContourSize(){return blackContours.size();}
}
