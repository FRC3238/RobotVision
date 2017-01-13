package org.usfirst.frc.team3238.robot;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Preferences;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.Collection;
import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;


public class ImageProcessor implements Runnable
{
    private static List<MatOfPoint> contours, preFilteredContours;
    
    private static double[] hThresh = new double[2];
    private static double[] sThresh = new double[2];
    private static double[] vThresh = new double[2];
    
    private static boolean external;
    
    private static double minArea = 0;
    private static double minPerimeter = 0;
    private static double minWidth = 0;
    private static double maxWidth = 1000;
    private static double minHeight = 0;
    private static double maxHeight = 1000;
    private static double[] solidity = { 0, 100 };
    private static double maxVertices = 1000000;
    private static double minVertices = 0;
    private static double minRatio = 0;
    private static double maxRatio = 1000;
    
    static Preferences prefs;
    
    static UsbCamera camera;
    static CvSink cvSink;
    static CvSource outputStream;
    static Mat source, output;
    
    public static void init()
    {
        prefs = Preferences.getInstance();
        hThresh[0] = prefs.getDouble("hThreshMin", 0.0);
        hThresh[1] = prefs.getDouble("hThreshmax", 255.0);
        sThresh[0] = prefs.getDouble("sThreshMin", 0.0);
        sThresh[1] = prefs.getDouble("sThreshMax", 255.0);
        vThresh[0] = prefs.getDouble("vThreshMin", 0.0);
        vThresh[1] = prefs.getDouble("vThreshMax", 255.0);
        external = prefs.getBoolean("external", true);
        minArea = prefs.getDouble("minArea", 0.0);
        minPerimeter = prefs.getDouble("minPerimeter", 0.0);
        minWidth = prefs.getDouble("minWidth", 0.0);
        maxWidth = prefs.getDouble("maxWidth", 1000);
        minHeight = prefs.getDouble("minHeight", 0.0);
        maxHeight = prefs.getDouble("maxHeight", 1000);
        solidity[1] = prefs.getDouble("soliditymax", 100);
        solidity[0] = prefs.getDouble("soliditymin", 0.0);
        maxVertices = prefs.getDouble("maxVertices", 1000000);
        minVertices = prefs.getDouble("minVertices", 0.0);
        minRatio = prefs.getDouble("minRatio", 0.0);
        maxRatio = prefs.getDouble("maxRatio", 1000);
        
        CameraServer.getInstance().startAutomaticCapture();
        camera.setResolution(640, 480);
        
        cvSink = CameraServer.getInstance().getVideo();
        outputStream = CameraServer.getInstance().putVideo("somethingThatIsActuallyGood", 640, 480);
        
        source = new Mat();
        output = new Mat();
    }
    
    public static List<MatOfPoint> getContours()
    {
        if(contours != null)
        {
            return contours;
        } else
        {
            return new List<MatOfPoint>()
            {
                @Override public int size()
                {
                    return 0;
                }
                
                @Override public boolean isEmpty()
                {
                    return false;
                }
                
                @Override public boolean contains(Object o)
                {
                    return false;
                }
                
                @Override public Iterator<MatOfPoint> iterator()
                {
                    return null;
                }
                
                @Override public Object[] toArray()
                {
                    return new Object[0];
                }
                
                @Override public <T> T[] toArray(T[] a)
                {
                    return null;
                }
                
                @Override public boolean add(MatOfPoint matOfPoint)
                {
                    return false;
                }
                
                @Override public boolean remove(Object o)
                {
                    return false;
                }
                
                @Override public boolean containsAll(Collection<?> c)
                {
                    return false;
                }
                
                @Override public boolean addAll(
                        Collection<? extends MatOfPoint> c)
                {
                    return false;
                }
                
                @Override public boolean addAll(int index,
                        Collection<? extends MatOfPoint> c)
                {
                    return false;
                }
                
                @Override public boolean removeAll(Collection<?> c)
                {
                    return false;
                }
                
                @Override public boolean retainAll(Collection<?> c)
                {
                    return false;
                }
                
                @Override public void clear()
                {
                    
                }
                
                @Override public MatOfPoint get(int index)
                {
                    return null;
                }
                
                @Override public MatOfPoint set(int index, MatOfPoint element)
                {
                    return null;
                }
                
                @Override public void add(int index, MatOfPoint element)
                {
                    
                }
                
                @Override public MatOfPoint remove(int index)
                {
                    return null;
                }
                
                @Override public int indexOf(Object o)
                {
                    return 0;
                }
                
                @Override public int lastIndexOf(Object o)
                {
                    return 0;
                }
                
                @Override public ListIterator<MatOfPoint> listIterator()
                {
                    return null;
                }
                
                @Override public ListIterator<MatOfPoint> listIterator(
                        int index)
                {
                    return null;
                }
                
                @Override public List<MatOfPoint> subList(int fromIndex,
                        int toIndex)
                {
                    return null;
                }
            };
        }
    }
    
    /**
     * When an object implementing interface <code>Runnable</code> is used
     * to create a thread, starting the thread causes the object's
     * <code>run</code> method to be called in that separately executing
     * thread.
     * <p>
     * The general contract of the method <code>run</code> is that it may
     * take any action whatsoever.
     *
     * @see Thread#run()
     */
    @Override public void run()
    {
        while(true)
        {
            cvSink.grabFrame(source);
            hsvThreshold(source, hThresh, sThresh, vThresh, output );
            findContours(output, external, preFilteredContours );
            filterContours(preFilteredContours, minArea,
                    minPerimeter, minWidth,
                    maxWidth, minHeight,
                    maxHeight, solidity,
                    maxVertices, minVertices,
                    minRatio, maxRatio,
                    contours);
            Imgproc.drawContours(source, contours, -1, new Scalar(255, 0,0));
            outputStream.putFrame(output);
        }
    }
    /**
        * Segment an image based on hue, saturation, and value ranges.
     *
     * @param input The image on which to perform the HSL threshold.
     * @param hue   The min and max hue
     * @param sat   The min and max saturation
     * @param val   The min and max value
     * @param out   The image in which to store the output.
     */
    private void hsvThreshold(Mat input, double[] hue, double[] sat,
            double[] val, Mat out)
    {
        Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2HSV);
        Core.inRange(out, new Scalar(hue[0], sat[0], val[0]),
                new Scalar(hue[1], sat[1], val[1]), out);
    }
    
    /**
     * Sets the values of pixels in a binary image to their distance to the
     * nearest black pixel.
     *
     * @param input    The image on which to perform the Distance Transform.
//     * @param type     The Transform.
//     * @param maskSize the size of the mask.
//     * @param output   The image in which to store the output.
     */
    private void findContours(Mat input, boolean externalOnly,
            List<MatOfPoint> contours)
    {
        Mat hierarchy = new Mat();
        contours.clear();
        int mode;
        if(externalOnly)
        {
            mode = Imgproc.RETR_EXTERNAL;
        } else
        {
            mode = Imgproc.RETR_LIST;
        }
        int method = Imgproc.CHAIN_APPROX_SIMPLE;
        Imgproc.findContours(input, contours, hierarchy, mode, method);
    }
    
    /**
     * Filters out contours that do not meet certain criteria.
     *
     * @param inputContours  is the input list of contours
     * @param output         is the the output list of contours
     * @param minArea        is the minimum area of a contour that will be kept
     * @param minPerimeter   is the minimum perimeter of a contour that will be
     *                       kept
     * @param minWidth       minimum width of a contour
     * @param maxWidth       maximum width
     * @param minHeight      minimum height
     * @param maxHeight      maximimum height
//     * @param Solidity       the minimum and maximum solidity of a contour
     * @param minVertexCount minimum vertex Count of the contours
     * @param maxVertexCount maximum vertex Count
     * @param minRatio       minimum ratio of width to height
     * @param maxRatio       maximum ratio of width to height
     */
    private void filterContours(List<MatOfPoint> inputContours, double minArea,
            double minPerimeter, double minWidth, double maxWidth,
            double minHeight, double maxHeight, double[] solidity,
            double maxVertexCount, double minVertexCount, double minRatio,
            double maxRatio, List<MatOfPoint> output)
    {
        final MatOfInt hull = new MatOfInt();
        output.clear();
        //operation
        for(int i = 0; i < inputContours.size(); i++)
        {
            final MatOfPoint contour = inputContours.get(i);
            final Rect bb = Imgproc.boundingRect(contour);
            if(bb.width < minWidth || bb.width > maxWidth)
                continue;
            if(bb.height < minHeight || bb.height > maxHeight)
                continue;
            final double area = Imgproc.contourArea(contour);
            if(area < minArea)
                continue;
            if(Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true)
                    < minPerimeter)
                continue;
            Imgproc.convexHull(contour, hull);
            MatOfPoint mopHull = new MatOfPoint();
            mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
            for(int j = 0; j < hull.size().height; j++)
            {
                int index = (int) hull.get(j, 0)[0];
                double[] point = new double[] { contour.get(index, 0)[0],
                        contour.get(index, 0)[1] };
                mopHull.put(j, 0, point);
            }
            final double solid = 100 * area / Imgproc.contourArea(mopHull);
            if(solid < solidity[0] || solid > solidity[1])
                continue;
            if(contour.rows() < minVertexCount
                    || contour.rows() > maxVertexCount)
                continue;
            final double ratio = bb.width / (double) bb.height;
            if(ratio < minRatio || ratio > maxRatio)
                continue;
            output.add(contour);
        }
    }
}
