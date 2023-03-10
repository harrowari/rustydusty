use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};
use std::usize;

#[derive(Copy, Clone, PartialEq, Eq, Hash)]
struct Node {
    x: i32,
    y: i32,
}

#[derive(Copy, Clone, PartialEq)]
struct State {
    cost: usize,
    heuristic: usize,
    node: Node,
}

impl Ord for State {
    fn cmp(&self, other: &State) -> Ordering {
        other
            .cost
            .cmp(&self.cost)
            .then_with(|| other.heuristic.cmp(&self.heuristic))
    }
}

impl PartialOrd for State {
    fn partial_cmp(&self, other: &State) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

fn heuristic(from: Node, to: Node) -> usize {
    ((from.x - to.x).abs() + (from.y - to.y).abs()) as usize
}

fn a_star(start: Node, goal: Node, obstacles: &[Node]) -> Option<Vec<Node>> {
    let mut heap = BinaryHeap::new();
    let mut costs = HashMap::new();
    let mut parents = HashMap::new();

    heap.push(State {
        cost: 0,
        heuristic: heuristic(start, goal),
        node: start,
    });
    costs.insert(start, 0);

    while let Some(State { cost, node, .. }) = heap.pop() {
        if node == goal {
            let mut path = vec![node];
            let mut current = node;
            while let Some(parent) = parents.get(&current) {
                path.push(*parent);
                current = *parent;
            }
            path.reverse();
            return Some(path);
        }

        for neighbor in &[
            Node { x: node.x + 1, y: node.y },
            Node { x: node.x - 1, y: node.y },
            Node { x: node.x, y: node.y + 1 },
            Node { x: node.x, y: node.y - 1 },
        ] {
            if obstacles.contains(&neighbor) {
                continue;
            }
            let next_cost = cost + 1;
            let heuristic = heuristic(*neighbor, goal);
            let state = State {
                cost: next_cost,
                heuristic,
                node: *neighbor,
            };
            let old_cost = costs.get(&neighbor).cloned().unwrap_or(usize::MAX);
            if next_cost < old_cost {
                heap.push(state);
                costs.insert(*neighbor, next_cost);
                parents.insert(*neighbor, node);
            }
        }
    }

    None
}

fn main() {
    let start = Node { x: 0, y: 0 };
    let goal = Node { x: 5, y: 5 };
    let obstacles = vec![
        Node { x: 1, y: 1 },
        Node { x: 2, y: 2 },
        Node { x: 3, y: 3 },
        Node { x: 4, y: 4 },
    ];

    if let Some(path) = a_star(start, goal, &obstacles) {
        for node in path {
            println!("({}, {})", node.x, node.y);
        }
    } else {
        println!("No path found");
    }
}

