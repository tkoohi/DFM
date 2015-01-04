

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point2d;

import org.jgrapht.util.FibonacciHeap;
import org.jgrapht.util.FibonacciHeapNode;

//import vgp.curve.geodesic.*;
/**==========================================================
% dfm - perform Dynamic Fast Marching.
% tahereh koohi 22 Tir 1389
% U=D; taw=P; 
*=========================================================*/


public class DFM_GameTheory_2d{
	 /* Global variables */
	 static int n;				// width
	 static int p;				// height
	 static double D [][];
	 static double W [][];
	 static double V [][];
	 public static double start_PT [];
	 public static double end_PT [];
	 static double[][] meshgrid_X;
	 static double[][] meshgrid_Y;
	 static int INITIAL_SIZE = 50;
     static List<Point_2d> existing_points;
     // Weighting parameters for planning (sets priority for being aggresive
     //	in the direction of the goal vs. avoiding obstacles


	 public static FibonacciHeap<Point_2d> open_heap;
	
	 static Point_2d goalpt;
	 static Point_2d startpt;
	 
	 
	 static double calculatekey(Point_2d x)
	 {
		Point_2d a=new Point_2d(x);
	 	return GW.GW_MIN(D[a.i][a.j],V[a.i][a.j]);
	 }
	
	 

	 private static void initialize()
	 {
		 D= new double [n][p];
		 V= new double [n][p];
	 	//initialize PTs
	 	for(int i=0;i<n;++i)
	 		for(int j=0;j<p;++j)
	 		{
	 			D[i][j]=GW.GW_INFINITE;
	 			V[i][j]=GW.GW_INFINITE;
	 		}
	 	int i=(int) start_PT[0];
	 	int j=(int) start_PT[1];
	 	startpt =new Point_2d(i,j);
	 	
	 	
	 	//FibonacciHeapNode<Point_2d> fib_Point_2d  =new FibonacciHeapNode<Point_2d>(pt_el,calculatekey(pt_el));

	 	
	 	i=(int) end_PT[0];
	 	j=(int) end_PT[1];
	 	goalpt =new Point_2d(i,j);
	 	V[i][j]=0;
	 	existing_points = new ArrayList<Point_2d>(INITIAL_SIZE);
	 	FibonacciHeapNode<Point_2d> fib_Point_2d  =new FibonacciHeapNode<Point_2d>(goalpt,calculatekey(goalpt));

	 	open_heap=new FibonacciHeap<Point_2d>();
	 	open_heap.insert(fib_Point_2d, 0);
	 	existing_points.add(goalpt);
	 	
	 	// add to heap
	 }
	
	 public static void SetDynamics(){
		 
		 /* Problem Parameters.*/
		 //%   targetRadius  Radius of target circle (positive).
		 //%   velocityA	  Speed of the evader (positive constant).
		 //%   velocityB	  Speed of the pursuer (positive constant).
		 //%   psi           Relative heading of the two vehicles.
		 //%   omega         Angular velocity in mode 2 -> curved motion.
		 int targetRadius = 5;
		 int velocityA = 3;
		 int velocityB = 4;
		 double psi = -4 * Math.PI / 3;
		 int omega = 1;
		 //% In this example, the target set is a circle at the origin
		 //%   that represents a collision in relative coordinates between the evader
		 //%   (player a, fixed at the origin facing right) and the pursuer (player b).

		 /*
		  *  Create the flow fields for straight motion.
		  *  Multiply by -1 to get forward PDE.
		  */
		 double[] straight ; 
		 straight= new double[2];
		 
		 /*   straight[0]=x dot   straight[1]=y dot   */
		 straight[0] = -(-velocityA + velocityB * Math.cos(psi)); 
		 straight[1] = -(velocityB * Math.sin(psi)) ;
		 
	 }
	 
		//Meshgrid function of Matlab. -see Matlab help
		void meshgrid(double StartX, double StartY)
		{
			double meshgridX_start =StartX;   //-width;
			double meshgridX_step = 1;

			double meshgridY_start = StartY;  //-height;
			double meshgridY_step = 1;

			for(int i=0; i<-StartX; i++){
				for(int j=0; j<-StartY; j++){
					meshgrid_X[i][j] = (meshgridX_start + 2*i*meshgridX_step)/(2*-StartX);
					meshgrid_Y[i][j] = (meshgridY_start + 2*j*meshgridY_step)/(2*-StartY);}}
		}
	 
	 public static void main(String[] args) {
		SetDynamics();
		Point2d[] start_end = LoadMap.PointRoiMap();
		n=LoadMap.W;
		p=LoadMap.H;
		
		System.out.println("The Map is loaded"); //Display the string.
		W= new double [n][p];
		start_PT=new double [2];
		end_PT=new double [2];
		
		for(int i=0;i<n;++i)
	 		for(int j=0;j<p;++j)
	 			W[i][j]=LoadMap.MapIp.getPixelValue(i, j);
		 /*meshgrid_X=new double[n][p];
		 meshgrid_Y=new double[n][p];*/
		 
		
		
		

		start_PT[0]=start_end[0].getX();
		start_PT[1]=start_end[0].getY();
		end_PT[0]=start_end[1].getX();
		end_PT[1]=start_end[1].getY();
		initialize();
		RunDFMRunnable_2d r = new RunDFMRunnable_2d(goalpt, open_heap, existing_points, W, V, D,LoadMap.W,LoadMap.H, startpt);
	    Thread t = new Thread(r);
		t.start();
		/*% display the distance function
		clf; imagesc(D);
		D(S==Inf)=0;    % remove Inf values that make contour crash
		figure; contour(D,50);
		FMMpath = compute_geodesic(D,end_point, options);*/
		
		/*PjGeodesic geodesic =new PjGeodesic();
		geodesic.init();*/
	}

	
	private  DFM_GameTheory_2d() {
	}


}
