package planner;

import java.awt.Rectangle;
import java.util.ArrayList;
import java.util.List;

public class World {
	protected int width, height;
	protected List<Rectangle> obstacles;
	protected List<Rectangle> walls;
	
	public World(int w, int h){
		width = w;
		height = h;
		obstacles = new ArrayList<Rectangle>();
		walls = new ArrayList<Rectangle>();
	}
	
	//add the obstacle with x y as the center
	//point and width as the edge
	public void addObs(int x, int y, int width){
		obstacles.add(new Rectangle(x, y, width, width));
	}
	public void addWall(Rectangle wall){
		walls.add(wall);
	}
	
	public List<Rectangle> getObstacles(){return obstacles;}
	public int getWidth(){return width;}
	public int getHeight(){return height;}

}
