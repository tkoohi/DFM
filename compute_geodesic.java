
/*
 * 
 *                        METHOD USING ODE
 * This routine solves a system of n nonlinear ordinary differential
equations

        d^2x^i/ds^2 + Gamma^i_{jk} dx^j/ds d^k/ds = 0,

where s is arc length, i = 1,2,...,n, and with implied summation on
j and k. Gamma is the Christoffel symbol of the second kind.
Boundary conditions are two geodesic endpoints. This routine uses a
relaxation (iterative) solving method.

There are two issues the user should understand. There can
be more than one minimum-length geodesic. This is common if
endpoints are located symmetrically on a symmetric geometry. This
routine approximates only one minimum-length geodesic.

This routine solves for any geodesic but returns the shortest curve
found. The distinction is important if the routine converges on a
geodesic other than a minimum-length geodesic. Success is indicated
by monitoring iteration statistics provided by compiler option
-D_DEBUG. Best results occur when convergence results in shorter
curves. If convergence is slow, the shortest curve found is the best
approximation of a minimum-length geodesic since a theorem proves an
actual shortest curve(s) is always a geodesic.

Reference:
    Manfredo do Carmo, Riemannian Geometry, Birkhauser, Boston, 1992
 * 
 */
public class compute_geodesic 
{
	public static double D [][];
	public static double Gx [][];
	public static double Gy [][];

	int h,w,n;
	double alpha=0.9;

	public Point_2d[] path = new Point_2d[n];
	
	/**
	 * computeGeodesic
	 */
	public compute_geodesic(double D [][],Point_2d goalPoint_2d, int h, int w, Point_2d startPoint_2d){


		this.D = D;
		
		this.h=h; this.w=w;
		computeGradient();
	
		path= GeneratePath(goalPoint_2d, h, w, startPoint_2d);

	

	}
	
	/**
	 * GeneratePath
	 */
	private Point_2d[] GeneratePath(Point_2d goalPoint_2d, int h, int w, Point_2d startPoint_2d){

		n=300;  // max iteration
		Point_2d[] xpos = new Point_2d[n];
		
		// 	path is empty
		xpos[0] = goalPoint_2d;

		int inew,jnew;
		inew=goalPoint_2d.i;
		jnew=goalPoint_2d.j;
		/* Update new position */
		int c=0;
		do{
			c++;
						
			xpos[c] = new Point_2d((int) (xpos[c-1].i-alpha*Gx[inew][jnew]),(int) (xpos[c-1].j-alpha*Gy[inew][jnew]));;
			if (c==n || c>=xpos.length)
				return xpos;
			
			inew = xpos[c].i;
			jnew = xpos[c].j;
		}while (xpos[c].i==startPoint_2d.i && xpos[c].j==startPoint_2d.j);
		return xpos;
		

	}
	
	
	/**
	 * computeGradient
	 */
	public void computeGradient(){
		double eps=0.0000009;
		Gx =new double [h][w];
		Gy =new double [h][w];
		double[][] d = new double [h][w];
	    for (int u = 0;u<h-2;u++)
	    	for (int v = 0;v<w-2;v++){
	    		// compute filter result for position (u,v)
	    		Gx[u][v]=gx(u,v);
	    		Gy[u][v]=gy(u,v);
	    		// normalize the gradient field
	    		d[u][v] = Math.sqrt( Gx[u][v]*Gx[u][v] + Gy[u][v]*Gy[u][v] );
	    		if (d[u][v]<eps)
	    			d[u][v]=1;
	    		Gx[u][v]=Gx[u][v]/d[u][v];
	    		Gy[u][v]=Gy[u][v]/d[u][v];
	    		
	    }
	    
	}
	
	
	/**
	 * Gx Sobel Operator
	 * @return 
	 */
	public double gx(int p, int q){
		return Math.abs(D[p+2][q]+2*D[p+2][q+1]+D[p+2][q+2])-(D[p][q]+2*D[p][q+1]+D[p][q+2]);}
	/**
	 * Gy Sobel Operator
	 * @return 
	 */
	public double gy(int p, int q){
		return Math.abs(D[p][q+2]+2*D[p+1][q+2]+D[p+2][q+2])-(D[p][q]+2*D[p+1][q]+D[p+2][q]);}
		

/**
 * Gx
 * @return 
 */
  /*public double gx(int p, int q){
	return (D[p+1][q]-D[p][q]);}*/
/**
 * Gy
 * @return 
 */
  /*public double gy(int p, int q){
	return (D[p][q+1]-D[p][q]);}*/
}


	    
		
	/*options = odeset('Events',@event_callback);
	[T,path] = ode113( @get_gradient, [0,Tmax], x, options);

	return;

	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% subfunction
	function [value,isterminal,direction] = event_callback(t,y)

	global startPoint_2d;
	% compute distance to start points    y(:,i) is the ith point living in R^d
	d = compute_distance_to_points(y,startPoint_2d);
	value = min(d);
	if value<0.1
	    value = -1;
	end
	isterminal = 1;
	direction = 0;

	
	function g = get_gradient( t, y )

	global grad;
	gx = grad(:,:,1);
	gy = grad(:,:,2);

	n = length(gx);

	% down/left corner of current cell
	p = floor(y(1));
	q = floor(y(2));
	    
	p = clamp(p,1,n-1);
	q = clamp(q,1,n-1);

	% residual in [0,1]
	xx = y(1)-p;
	yy = y(2)-q;

	xx = clamp(xx,0,1);
	yy = clamp(yy,0,1);

	% compute gradient    
	a = [gx(p,q), gy(p,q)];
	b = [gx(p+1,q), gy(p+1,q)];
	c = [gx(p,q+1), gy(p,q+1)];
	d = [gx(p+1,q+1), gy(p+1,q+1)];     
	g = ( a*(1-xx)+b*xx )*(1-yy) + ( c*(1-xx)+d*xx )*yy;
	g = -g';






	function path = extract_path_3d(D,end_points,options)

	% extract_path_3d - extract the shortest path using 
	%   a gradient descent.
	%
	%   path = extract_path_3d(D,x,options);
	%
	%   'D' is the distance function.
	%   'x' is starting point (should be integer). 
	%
	%   Copyright (c) 2004 Gabriel Peyrژ


	options.null = 0;

	if isfield(options, 'trim_path')
	    trim_path = options.trim_path;
	else
	    trim_path = 1;
	end

	% gradient computation
	I = find(D==Inf);
	J = find(D~=Inf);
	D1 = D; D1(I) = mmax(D(J));
	global gx;
	global gy;
	global gz;
	[gy,gx,gz] = gradient(D1);

	% normalize the gradient field
	d = sqrt( gx.^2 + gy.^2 + gz.^2 );
	I = find(d<eps);
	d(I) = 1;
	gx = gx./d; gy = gy./d; gz = gz./d;

	% path extraction
	options = [0.2 20000];
	path = stream3(-gy,-gx,-gz,end_points(2,:),end_points(1,:),end_points(3,:), options);
	for i=1:length(path)
	     path{i} = path{i}(:,[2:-1:1 3]);
	end
	if length(path)==1
	    path = path{1};
	end

	% test if path is long enough
	v = path(end,:);
	v1 = max(round(v),ones(1,3));
	if( 0 && A1(v1(1),v1(2),v1(3))>0.1 )
	    path1 = stream3(-gy,-gx,-gz,v(2),v(1),v(3), options);
	    for i=1:length(path1)
	        path1{i} = path1{i}(:,[2:-1:1 3]);
	    end
	    if length(path)>=1
	        path1 = path1{1};
	    end
	    path = [path; path1];
	end

	if isfield(options, 'startPoint_2d')
	    startPoint_2d = options.startPoint_2d;
	else
	    startPoint_2d = path(end,:);
	end
	startPoint_2d = startPoint_2d(:);

	if trim_path
	    % removing too verbose points
	    d = compute_distance_to_points(path', startPoint_2d);
	    % perform thresholding
	    T = mmax(d)/300^2;
	    I = find(d<T);
	    if ~isempty(I)
	        path = path(1:I(1), :);
	        path = [path; startPoint_2d'];
	    end
	end

	

*/
