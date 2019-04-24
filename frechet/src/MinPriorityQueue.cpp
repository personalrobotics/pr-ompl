#include "MinPriorityQueue.h"

void MinPriorityQueue::update(Graph &g, Vertex &v, double newPriority) {
  IndexMap index_map = get(boost::vertex_index, g);
  mMinHeap.update(index_map[v], newPriority);
}

void MinPriorityQueue::insert(Graph &g, Vertex &v, double priority) {
  IndexMap index_map = get(boost::vertex_index, g);
  mMinHeap.insert(index_map[v], priority);
}

void MinPriorityQueue::remove(Graph &g, Vertex &v) {
  IndexMap index_map = get(boost::vertex_index, g);
  mMinHeap.remove(index_map[v]);
}

bool MinPriorityQueue::contains(Graph &g, Vertex &v) {
  IndexMap index_map = get(boost::vertex_index, g);
  return mMinHeap.contains(index_map[v]);
}

bool MinPriorityQueue::isEmpty() { return (mMinHeap.size() == 0); }

double MinPriorityQueue::peek() { return mMinHeap.top_key(); }

Vertex MinPriorityQueue::pop(Graph &g) {
  Vertex top_vertex = vertex(mMinHeap.top_idx(), g);

  mMinHeap.remove_min();
  return top_vertex;
}
