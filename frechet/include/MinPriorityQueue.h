#ifndef MIN_PRIORITY_QUEUE_
#define MIN_PRIORITY_QUEUE_

#include "heap_indexed.h"
#include "util.hpp"

using namespace pr_bgl;

typedef boost::property_map<Graph, boost::vertex_index_t>::type IndexMap;

/// A minimum priority queue backed by a min-heap.
class MinPriorityQueue {

public:
  MinPriorityQueue() {}

  /// Update the priority of a node already on the PQ.
  ///
  /// \param[in] g Graph the node belongs to.
  /// \param[in] v Node to update the priority of.
  /// \param[in] newPriority New priority of the node.
  void update(Graph &g, Vertex &v, double newPriority);

  /// Insert a new node into the priority queue with given priority.
  ///
  /// \param[in] g Graph the node belongs to.
  /// \param[in] v Node to insert into the PQ.
  /// \param[in] priority Priority of the node.
  void insert(Graph &g, Vertex &v, double priority);

  /// Delete a node from the priority queue.
  ///
  /// \param[in] g Graph the node belongs to.
  /// \param[in] v Node to delete from the PQ.
  void remove(Graph &g, Vertex &v);

  /// Check if the priority queue contains a given node.
  ///
  /// \param[in] g Graph the node belongs to.
  /// \param[in] v Node to check for in PQ.
  bool contains(Graph &g, Vertex &v);

  /// Return true if the priority queue contains no nodes.
  bool isEmpty();

  /// Get the smallest priority value in the queue.
  double peek();

  /// Remove and return the node with the smallest priority in the PQ.
  ///
  /// \param[in] g Graph the search is on.
  Vertex pop(Graph &g);

private:
  /// Min-heap that backs this priority queue.
  heap_indexed<double> mMinHeap;
}; // class MinPriorityQueue

#endif // MIN_PRIORITY_QUEUE_
