package planner;

import java.util.Arrays;

public class Arm implements Comparable<Arm>{
	//base x, y
	protected double[] base;
	//links' length
	protected double[] link_len;
	//links' angles
	protected double[] link_ang;
	//number of links
	protected int link_num;
	//width of link
	protected double width;
	//Goal
	protected double[] goal_ang;
	
	protected double cost;
	public Arm(int num, double baseX, double baseY){
		link_num = num;
		base = new double[2];
		base[0] = baseX;
		base[1] = baseY;
		link_len = new double[link_num];
		link_ang = new double[link_num];
		for(int i = 0; i < link_num; i++){
			link_len[i] = 0;
			link_ang[i] = 0;
		}
		width = 10;
		cost = 0;
	}
	
	
	public void setLink(int i, double len, double ang){
		link_len[i] = len;
		link_ang[i] = ang;
	}
	public void setBase(double x, double y){
		base[0] = x;
		base[1] = y;
	}
	public void setWidth(double _width){width = _width;}
	public void setConfig(double[] configs){
		for(int i = 0; i < link_num; i++)
			link_ang[i] = configs[i];
	}
	public void setConfig(double config, int i){
		link_ang[i] = config;
	}
	public int getLinkNum(){return link_num;}
	public double getLinkLen(int i){return link_len[i];}
	public double getLinkAng(int i){return link_ang[i];}
	public double[] getLinkAng(){return link_ang;}
	public double[] getBase(){return base;}
	
	//Get the rectangle (coordinates from four vertices)
	public double[][]getRec(int i){
		double[][] rect = new double[4][2];
		
		double x = base[0];
		double y = base[1];
		double ang = 0;
		//the first link point of the link
		for(int j = 0; j < i; j++){
			ang = (ang + link_ang[j]) % (2 * Math.PI);
			x = x + link_len[j] * Math.cos(ang);
			y = y + link_len[j] * Math.sin(ang);
		}
		//the second link point of the link
		ang = (ang + link_ang[i]) % (2 * Math.PI);
		double x_next = x + link_len[i] * Math.cos(ang);
		double y_next = y + link_len[i] * Math.sin(ang);
		
		//Get the rectangles four vertices' coordinates, we can think of
		//the edge as another arm with length = width / 2. The order of the 
		//vertices are counter-clockwise, which are, top-left, top-right, 
		//bottom-right and bottom left of the rectangle.
		rect[0][0] = x_next + width / 2 * Math.cos(ang + Math.PI / 2);
		rect[0][1] = y_next + width / 2 * Math.sin(ang + Math.PI / 2);
		rect[1][0] = x_next + width / 2 * Math.cos(ang + Math.PI * 1.5);
		rect[1][1] = x_next + width / 2 * Math.sin(ang + Math.PI * 1.5);
		rect[2][0] = x + width / 2 * Math.cos(ang + Math.PI * 1.5);
		rect[2][1] = x + width / 2 * Math.sin(ang + Math.PI * 1.5);
		rect[3][0] = x + width / 2 * Math.cos(ang + Math.PI / 2);
		rect[3][1] = y + width / 2 * Math.sin(ang + Math.PI / 2);

		return rect;
	}
	
	@Override
	public int compareTo(Arm arg0) {
		// TODO Auto-generated method stub
		return (int) Math.signum(priority() - arg0.priority());
	}
	@Override
	public boolean equals(Object o){
		return Arrays.equals(link_ang, ((Arm) o).getLinkAng());
	}
	@Override
	public int hashCode(){return link_ang.hashCode();}
	
	public double priority(){
		return heuristic() + getCost();
	}
	public double heuristic(){
		double h = 0;
		for(int i = 0; i < link_num; i++){
			h += Math.abs(link_ang[i] - goal_ang[i]);
		}
		return h;
	}
	public double manhattan(double ang1, double ang2){
		double res = Math.abs(ang1 - ang2);
		if(res > Math.PI)
			return 2 * Math.PI - res;
		else return res;
	}
	public double getCost(){return cost;}
	public void setCost(double c){cost = c;}
	public void setGoal(double[] angles){link_ang = angles;}
}
