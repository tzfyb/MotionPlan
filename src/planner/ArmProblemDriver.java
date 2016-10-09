package planner;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Polygon;
import java.awt.Rectangle;
import java.util.List;

import javax.swing.BorderFactory;
import javax.swing.BoxLayout;
import javax.swing.JFrame;
import javax.swing.JPanel;

import planner.ArmProblem.ArmProblemNode;
import planner.SearchProblem.SearchNode;

public class ArmProblemDriver extends JFrame implements Runnable {
	private static final long serialVersionUID = 1L;
	protected World world;
	protected ArmProblem armRobot;
	protected List<SearchNode> path;
	protected int curStep;
	protected int totalSteps;

	public static void main(String[] args) {
		ArmProblemDriver apd = new ArmProblemDriver();
		apd.run();
	}

	public ArmProblemDriver() {
		// Create the world
		world = new World(300, 300);
		world.addObs(55, 55, 40);
		world.addObs(205, 55, 40);
		world.addObs(55, 205, 40);
		world.addObs(205, 205, 40);

		// Create ArmRobot and get the path
		double[] start = { 0, 0, 0};
		//double[] start = {0.7250157773431886, 0.4678358522216472};
		double[] goal = { Math.PI, 0, 0};
		double[] base = {world.width / 2, world.height / 2};
		armRobot = new ArmProblem(3, start, goal, base, world, 5, 500, 0.1, 2016);
		//armRobot.setBase(world.width / 2, world.height / 2);
		//path = armRobot.astarSearch();
		List<SearchNode> oriPath = armRobot.astarSearch();
		if(oriPath == null){
			System.out.println("Path does not exist");
			System.exit(0);
		}
		
		path = armRobot.smoothPath(oriPath);
		//path = new ArrayList<SearchNode>();
		//path.add(armRobot.getStart());
		//path.add(armRobot.getGoal());

		curStep = -1;
		totalSteps = path.size();

		// Prepare the JPanel
		MyPanel panel = new MyPanel();
		this.add(panel);

		// Set the jframe size and title
		this.setTitle("Arm Motion Plan");
		this.getContentPane().setLayout(new BoxLayout(this.getContentPane(), BoxLayout.Y_AXIS));
		this.setSize(500, 500);
		this.setResizable(false);
		this.setLocationRelativeTo(null);
		this.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		this.setVisible(true);

	}

	public class MyPanel extends JPanel {
		private static final long serialVersionUID = 1L;

		@Override
		public void paint(Graphics g) {
			super.paint(g);
			Dimension expectedDimension = new Dimension(world.width, world.height);
			this.setPreferredSize(expectedDimension);
			this.setMinimumSize(expectedDimension);
			this.setMaximumSize(expectedDimension);
			this.setBorder(BorderFactory.createLineBorder(Color.BLACK));
			this.setBackground(Color.gray);

			g.setColor(Color.BLACK);
			for (Rectangle rect : world.getObstacles()) {
				g.fillRect(rect.x, rect.y, rect.width, rect.height);
			}

			List<Polygon> arms = ((ArmProblemNode)(path.get(curStep))).getAllPoly(true);
			
			if(((ArmProblemNode)(path.get(curStep))).armCollision(world))
				g.setColor(Color.RED);
			else
				g.setColor(Color.LIGHT_GRAY);
			for(Polygon arm : arms){
				g.drawPolygon(arm);
			}
			g.setColor(Color.RED);
			g.drawOval(world.width / 2 - 3, world.height / 2 - 3, 6, 6);
		}

	}

	@Override
	public void run() {
		// TODO Auto-generated method stub
		while (true) {
			curStep = (curStep + 1) % totalSteps;
			try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			System.out.println("Step: " + Integer.toString(curStep) + "/" + Integer.toString(path.size()));
			if(((ArmProblemNode)(path.get(curStep))).armCollision(world))
				System.out.println("Intersect!");
			System.out.println(((ArmProblemNode)(path.get(curStep))).toString());

			repaint();
		}
	}
}
