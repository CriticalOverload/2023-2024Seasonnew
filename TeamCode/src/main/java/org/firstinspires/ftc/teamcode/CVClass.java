package org.firstinspires.ftc.teamcode;


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

public class CVClass extends OpenCvPipeline{
    public static double pdiffs = 30;
    private double boxPDiffs = 0.0;
    private List<MatOfPoint> blackContours;
    Mat output = new Mat();//output mat

    Rect blackbox, greenbox;//draws box

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
        input.copyTo(output);
        List<MatOfPoint> greenContours = getGreen(input);
        blackContours = getBlack(input);//may make this black?


        //need to draw the rectangles first...

        if(greenContours.size() > 0){
            double max = 0;
            int maxInd = 0;
            for(int i = 0; i < greenContours.size(); i++){//loop through all the contours, and find the largest box
                double area = Imgproc.contourArea(greenContours.get(i));
                if(area > max){
                    max = area;
                    maxInd = i;
                }
            }
            //draw a box of the largest one of single color
            Rect largestRect = Imgproc.boundingRect(greenContours.get(maxInd));
            greenbox = largestRect;
            Scalar boxColor = new Scalar(38, 255, 0);//not white... something!!
            Imgproc.rectangle(output, greenbox, boxColor, 3, 8, 0);//Currently boxed based on rectangle
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

        if(greenContours.size() > 0){//in other words, it found green
            if(blackContours.size() > 0){//found black... change height!!!
                boxPDiffs = 200.0*Math.abs(blackbox.height-greenbox.height)/(double)(blackbox.height+ greenbox.height);//percent difference: |a-b|/((a+b)/2)*100=200*|a-b|/(a+b)//todo use area?
                double boxDists = Math.sqrt(Math.pow(blackbox.x-greenbox.x,2)+Math.pow(blackbox.y-greenbox.y,2));
                if (boxPDiffs < pdiffs && boxDists < 20){
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
    //better one:
    //https://colorpicker.me/#8a2b48
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
        double hue = 80;
        double sensitivity = 15;//to adjust for slight variations. Ewww that's a giant sensitivity...

//        Scalar lowBound = new Scalar((hue/2)-sensitivity,100,50);//lower bound... may be wrong???
        Scalar lowBound = new Scalar(hue/2-sensitivity,90,127);//lower bound
        Scalar hiBound = new Scalar(hue/2+sensitivity,255,255);//higher bound
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
        double satlow = 0;
        double sathi = 255;
        double vallow = 0;
        double valhi = 30;
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
