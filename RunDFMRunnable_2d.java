/**
*
* @ author: tahereh koohi
* tahereh.koohi@gmail.com
* intelligent-karoon.rozblog.com
*
**/


import java.util.List;

import ij.IJ;
import ij.ImagePlus;
import ij.process.ByteProcessor;
import ij.process.ImageProcessor;

import org.jgrapht.util.FibonacciHeap;
import org.jgrapht.util.FibonacciHeapNode;

class RunDFMRunnable_2d implements Runnable
{
	 /* Global variables */
	 int n;				// width
	 int p;				// height
	 public static double D [][];
	 public static double V [][];
	 public FibonacciHeap<Point_2d> open_heap;
	 public List<Point_2d>  existing_points;
	 double W [][];
	 double h=1.0;
	 public Point_2d goalPoint_2d;
	 public Point_2d  startPoint_2d;
	 int Nsteps=500;            // Maximum number of steps to produce

	public RunDFMRunnable_2d(Point_2d goalPoint_2d, FibonacciHeap<Point_2d> open_heap,List<Point_2d>  existing_points, double W [][], double V [][], double D [][], int n, int p, Point_2d startPoint_2d){
		this.goalPoint_2d=goalPoint_2d;
		this.startPoint_2d=startPoint_2d;
		this.n=n;
		this.p=p;
		this.open_heap=open_heap;
		this.existing_points=existing_points;
		RunDFMRunnable_2d.V=V;
		RunDFMRunnable_2d.D=D;
		this.W=W;
		
		
	}
	public void run()
   {
      //forever
	/** Start the simulation loop**/ 
	for (int k=1; k<Nsteps; k++){
	 
	 RunDFM();
	 Sleep(10);
	
	 //SenseCircularPattern sense = new SenseCircularPattern(startPoint_2d);
	 new RayCasting(startPoint_2d, LoadMap.W, LoadMap.H, W);
	 CreateMap Map = new CreateMap(RayCasting.s_array);
	 
	 compute_geodesic geodesic = new compute_geodesic(RunDFMRunnable_2d.D,goalPoint_2d,LoadMap.W,LoadMap.H, startPoint_2d);

	 System.out.println("The Path is generated"); //Display the string.

		
	 ImageProcessor D_Ip = new ByteProcessor(n, p);
	 ImagePlus D_Im = new ImagePlus("D_Image", D_Ip);
	 for(int i=0;i<n;++i)
	 	for(int j=0;j<p;++j)
	 		D_Ip.putPixelValue(i, j, D[i][j]);
			
	 for (int l=0;l<geodesic.path.length;l++){
		 System.out.println(geodesic.path[l].j); //Display the string.
		 int i= geodesic.path[l].i;
	 	 int j= geodesic.path[l].j;
	 	 D_Ip.putPixelValue(i, j, D[i][j]);}
			
	 IJ.save(D_Im, "D:\\eclipse\\dfm\\src\\models\\D_image"+".png");
	 D_Im.close();		 
	 
	 //Wait for changes in taw;
	 //if(there is any configuration with changed cost)
	 //Update taw({x})
	 //Update({x})
	 //else{
	 //try{
	 //Thread.sleep(500);}
	 //catch(){
	 //e.printStackTrace();}}
	 System.out.println(k); //Display the string.

	}
	System.exit(1);
   }
	
	/**
	 * Sleep is a utility function that encapsulates the necessary try/catch.
	 * 
	 */
	public static void Sleep(int millis) {
		try {
			Thread.sleep(millis);
		} catch (InterruptedException e) {
			System.err.println("Insomnia!");
			e.printStackTrace();
		}
	}
	/**
	 * 
	 * RunDFM
	 */
	 void RunDFM()
	 {
	 	double minKey;
		if (open_heap.isEmpty())
	 		minKey=GW.GW_INFINITE;
	 	else
	 		minKey=open_heap.min().getKey();
	 	//Point_2d a=new Point_2d(startPoint_2d);
	 	
	 	Point_2d cur_Point_2d=new Point_2d(startPoint_2d);
	 	/*System.out.println(minKey<calculatekey(startPoint_2d));
	 	System.out.println(minKey+","+calculatekey(startPoint_2d));
	 	System.out.println(Math.abs(D[a.i][a.j]-V[a.i][a.j])>GW.TOL);
	 	System.out.println("Err:"+" i:"+a.i+" j:"+a.j+"  "+Math.abs(D[a.i][a.j]-V[a.i][a.j])+" tol:"+GW.TOL+"\n");*/
	 	while(minKey<calculatekey(cur_Point_2d) || Math.abs(D[cur_Point_2d.i][cur_Point_2d.j]-V[cur_Point_2d.i][cur_Point_2d.j])> GW.TOL)
	 	{
	 		System.out.println(minKey<calculatekey(startPoint_2d));
		 	System.out.println(minKey+","+calculatekey(startPoint_2d));
		 	System.out.println(Math.abs(D[cur_Point_2d.i][cur_Point_2d.j]-V[cur_Point_2d.i][cur_Point_2d.j])>GW.TOL);
		 	System.out.println("Err:"+" i:"+cur_Point_2d.i+" j:"+cur_Point_2d.j+"  "+Math.abs(D[cur_Point_2d.i][cur_Point_2d.j]-V[cur_Point_2d.i][cur_Point_2d.j])+" tol:"+GW.TOL+"\n");
	 		
	 		//System.out.println("minKey:"+open_heap.min().getKey()+"\n");
	 		
	 		/*
	 		 * Q.pop(current Point_2d)
	 		 * 
	 		 */
		 	cur_Point_2d=(Point_2d) open_heap.min().getData();
	 		open_heap.removeMin();
	 		existing_points.remove(cur_Point_2d);
	 		int i=cur_Point_2d.i;
	 		int j=cur_Point_2d.j;
	 		int nei_i[]={i-1,i,i+1,i};
	 		int nei_j[]={j,j+1,j,j-1};
	 		if (V[i][j]<D[i][j])
	 		{
	 			D[i][j]=V[i][j];
	 			/**
	 			 *  for all Neigh(x) update(Neigh(x))
	 			 */
	 			for(int k=0;k<4;++k)
	 			{
	 				int ii=nei_i[k];
	 				int jj=nei_j[k];
	 				/**
	 				 *  check that the constraint distance map is ok
	 				 */
	 				if(ii>=0 && jj>=0 && ii<n && jj<p){
	 					update(ii,jj);
	 				    System.out.println("<"+ii+"+"+jj); //Display the string.
	 				}
	 			}
	 		}
	 		else
	 		{
	 			D[i][j]=GW.GW_INFINITE;
	 			/**
	 			 *  for all Neigh(x) union {x} update
	 			 */
	 			update(i,j);
	 			for(int k=0;k<4;++k)
	 			{
	 				int ii=nei_i[k];
	 				int jj=nei_j[k];
	 				/**
	 				 * check that the constraint distance map is ok
	 				 */
	 				if(ii>=0 && jj>=0 && ii<n && jj<p){
	 					update(ii,jj);
	 				    System.out.println(">"+ii+"+"+jj); //Display the string.
	 			}
	 			}

	 		}
			if (open_heap.isEmpty())
		 		minKey=GW.GW_INFINITE;
		 	else
		 		minKey=open_heap.min().getKey();	
	 	}//while
	 }//RunDFM()
	
	 
	 
	 void update(int i, int j)
	 {
	 	if (i!=startPoint_2d.i && j!=startPoint_2d.j)
	 		V[i][j]=FMComputeV(i,j);
	 	Point_2d Point_2d_el =new Point_2d(i,j);
	 	FibonacciHeapNode<Point_2d> fib_Point_2d  =new FibonacciHeapNode<Point_2d>(Point_2d_el,calculatekey(Point_2d_el));
	 	if (existing_points.contains(Point_2d_el))
	 		open_heap.delete(fib_Point_2d);
	 	System.out.println("not inserted");
	 	if(Math.abs(D[i][j]-V[i][j])>GW.TOL){
	 		open_heap.insert(fib_Point_2d, calculatekey(Point_2d_el));
	 		System.out.println("inserted"+calculatekey(Point_2d_el));
		 	existing_points.add(Point_2d_el);

		 	
	 	}

	 	// add to heap

	 }
	 
	 double FMComputeV(int i , int j)
	 {
	 	int nei_i[]={i-1,i,i+1,i};
	 	int nei_j[]={j,j+1,j,j-1};
	 	double D_alpha= GW.GW_MIN(D[nei_i[0]][nei_j[0]],D[nei_i[2]][nei_j[2]]);
	 	double D_beta= GW.GW_MIN(D[nei_i[1]][nei_j[1]],D[nei_i[3]][nei_j[3]]);
	 	double D_A= GW.GW_MIN(D_alpha,D_beta);
	 	double D_B= GW.GW_MAX(D_alpha,D_beta);
	 	if (D_A < GW.GW_INFINITE || D_B<GW.GW_INFINITE){
	 		//dastoor zir jash inja nist
	 		double taw=h/W[i][j];
	 		if (taw>(D_B-D_A)){
	 			double delta=2*taw*taw-(D_B-D_A)*(D_B-D_A);
	 			V[i][j]=(D_A+D_B+ Math.sqrt(delta))/2.0;
	 		}
	 		else
	 			V[i][j]=D_A+taw;
	 		System.out.println("V[i][j]:"+V[i][j]+" D_A:"+D_A+" D_B:"+D_B+" taw:"+taw);
	 	}
	 	
	 	else
	 		V[i][j]=GW.GW_INFINITE;

	 	return V[i][j];
	 }
	 
	 static double calculatekey(Point_2d x){
	 	Point_2d a=new Point_2d(x);
	 	return GW.GW_MIN(D[a.i][a.j],V[a.i][a.j]);
	 }
}

