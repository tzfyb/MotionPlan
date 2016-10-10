package planner;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Queue;

public abstract class SearchProblem {
	// Stats variable, mostly for debugging
	protected int nodesExplored;
	protected int maxMemory;

	protected SearchNode startNode;

	protected interface SearchNode extends Comparable<SearchNode> {

		// Every part of the assignment will need getSuccessors method
		public ArrayList<SearchNode> getSuccessors();

		public boolean goalTest();

		// cost in A* search and uniform search
		public double getCost();

		// return the heuristic
		public double getHeuristic();

		// return the priority, which is heuristic + cost
		public double getPriority();
	}

	public List<SearchNode> breadthFirstSearch() {
		resetStats();

		HashMap<SearchNode, SearchNode> visited = new HashMap<SearchNode, SearchNode>();
		Queue<SearchNode> my_queue = new LinkedList<SearchNode>();

		my_queue.add(startNode);
		// Keep on visit nodes till goal found or all nodes visited
		while (!my_queue.isEmpty()) {
			SearchNode cur = my_queue.poll();
			// If current node is goal, return path
			if (cur.goalTest()) {
				updateMemory(visited.size() + 1);
				nodesExplored = visited.size() + 1;
				// incrementNodeCount();
				return backchain(cur, visited);
			}
			// If current node is not goal, keep on searching
			ArrayList<SearchNode> suc = cur.getSuccessors();
			for (SearchNode s : suc) {
				if (!visited.containsValue(s)) {
					visited.put(s, cur);
					my_queue.add(s);
				}
			}
		}
		return null;
	}

	// backchain for bfs
	private List<SearchNode> backchain(SearchNode node, HashMap<SearchNode, SearchNode> visited) {
		// you will write this method
		List<SearchNode> res_path = new ArrayList<SearchNode>();
		res_path.add(node);
		while (node != startNode) {
			node = visited.get(node);
			res_path.add(node);
		}
		Collections.reverse(res_path); // Make res_path be in the right order
		return res_path;
	}

	public List<SearchNode> astarSearch() {
		resetStats();

		PriorityQueue<SearchNode> my_pq = new PriorityQueue<SearchNode>();
		HashMap<SearchNode, Double> visited = new HashMap<SearchNode, Double>();
		HashMap<SearchNode, SearchNode> parent = new HashMap<SearchNode, SearchNode>();

		my_pq.add(startNode);
		// visited.put(startNode, startNode.getPriority());

		while (!my_pq.isEmpty()) {
			updateMemory(my_pq.size() + parent.size());
			incrementNodeCount();
			SearchNode cur = my_pq.poll();
			// If a same node with higher priority exists, discard the node
			if (visited.containsKey(cur) && visited.get(cur) <= cur.getPriority())
				continue;
			else
				visited.put(cur, cur.getPriority());

			// Goal reached, return the result chain
			if (cur.goalTest())
				return backchain(cur, parent);

			ArrayList<SearchNode> suc = cur.getSuccessors();
			for (SearchNode s : suc) {
				if (!visited.containsKey(s) || visited.get(s) > s.getPriority()) {
					parent.put(s, cur);
					my_pq.add(s);
				}
			}
		}
		return null;
	}

	protected void resetStats() {
		nodesExplored = 0;
		maxMemory = 0;
	}

	protected void printStats() {
		System.out.println("  Nodes explored during search:  " + nodesExplored);
		System.out.println("  Maximum space usage during search " + maxMemory);
	}

	protected void updateMemory(int currentMemory) {
		maxMemory = Math.max(currentMemory, maxMemory);
	}

	protected void incrementNodeCount() {
		nodesExplored++;
	}
}
