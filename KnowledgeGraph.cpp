#include "KnowledgeGraph.h"

// =============================================================================
// Class Edge Implementation
// =============================================================================

template <class T>
Edge<T>::Edge(VertexNode<T>* from, VertexNode<T>* to, float weight) {
    this->from = from;
    this->to = to;
    this->weight = weight;
}

template <class T>
string Edge<T>::toString() {
    // TODO: Return the string representation of the edge
    return "";
}

// TODO: Implement other methods of Edge:


// =============================================================================
// Class VertexNode Implementation
// =============================================================================

template <class T>
VertexNode<T>::VertexNode(T vertex, bool (*vertexEQ)(T&, T&), string (*vertex2str)(T&)) {
    this->vertex = vertex;
    this->vertexEQ = vertexEQ;
    this->vertex2str = vertex2str;
    this->inDegree_ = 0;
    this->outDegree_ = 0;
}

template <class T>
void VertexNode<T>::connect(VertexNode<T>* to, float weight) {
    // TODO: Connect this vertex to the 'to' vertex
}

// =============================================================================
// Class DGraphModel Implementation
// =============================================================================

template <class T>
DGraphModel<T>::DGraphModel(bool (*vertexEQ)(T&, T&), string (*vertex2str)(T&)) {
    this->vertexEQ = vertexEQ;
    this->vertex2str = vertex2str;
}

template <class T>
DGraphModel<T>::~DGraphModel() {
    // TODO: Clear all vertices and edges to avoid memory leaks
}

template <class T>
void DGraphModel<T>::add(T vertex) {
    // TODO: Add a new vertex to the graph
}

template <class T>
void DGraphModel<T>::connect(T from, T to, float weight) {
    // TODO: Connect two vertices 'from' and 'to'

}

// TODO: Implement other methods of DGraphModel:

// =============================================================================
// Class KnowledgeGraph Implementation
// =============================================================================

KnowledgeGraph::KnowledgeGraph() {
    // TODO: Initialize the KnowledgeGraph
}

void KnowledgeGraph::addEntity(string entity) {
    // TODO: Add a new entity to the Knowledge Graph
}

void KnowledgeGraph::addRelation(string from, string to, float weight) {
    // TODO: Add a directed relation
}

// TODO: Implement other methods of KnowledgeGraph:



// =============================================================================
// Explicit Template Instantiation
// =============================================================================

template class Edge<string>;
template class Edge<int>;
template class Edge<float>;
template class Edge<char>;

template class VertexNode<string>;
template class VertexNode<int>;
template class VertexNode<float>;
template class VertexNode<char>;

template class DGraphModel<string>;
template class DGraphModel<int>;
template class DGraphModel<float>;
template class DGraphModel<char>;