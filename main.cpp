#include "KnowledgeGraph.h"

// =============================================================================
// Simple Testing Framework
// =============================================================================
int passedTests = 0;
int totalTests = 0;

template<typename T>
void assertEqual(T actual, T expected, string testName) {
    totalTests++;
    if (actual == expected) {
        cout << "[PASS] " << testName << endl;
        passedTests++;
    } else {
        cout << "[FAIL] " << testName << endl;
        cout << "       Expected: " << expected << endl;
        cout << "       Actual:   " << actual << endl;
    }
}

void assertTrue(bool condition, string testName) {
    assertEqual(condition, true, testName);
}

// Helpers for DGraphModel testing
string int2str(int& i) { return to_string(i); }
bool intEQ(int& a, int& b) { return a == b; }

// =============================================================================
// Test Scenarios
// =============================================================================

void testLowLevelGraph() {
    cout << "\n=== Testing Low-Level DGraphModel (Integer Graph) ===" << endl;
    DGraphModel<int> intGraph(intEQ, int2str);

    intGraph.add(1);
    intGraph.add(2);
    assertTrue(intGraph.contains(1), "Graph contains vertex 1");
    
    intGraph.connect(1, 2, 5.5f);
    assertTrue(intGraph.connected(1, 2), "Vertex 1 connected to 2");
    assertEqual(intGraph.weight(1, 2), 5.5f, "Edge weight 1->2 is 5.5");
    
    intGraph.disconnect(1, 2);
    assertTrue(!intGraph.connected(1, 2), "Vertex 1 disconnected from 2");
}

void testKnowledgeGraphBasics() {
    cout << "\n=== Testing KnowledgeGraph Wrapper ===" << endl;
    KnowledgeGraph kg;

    kg.addEntity("Alice");
    kg.addEntity("Bob");
    kg.addRelation("Alice", "Bob", 1.0);

    assertTrue(kg.isReachable("Alice", "Bob"), "Alice -> Bob is reachable");
    
    try {
        kg.addEntity("Alice");
        cout << "[FAIL] Should have thrown EntityExistsException" << endl;
    } catch (...) {
        cout << "[PASS] Threw Exception on duplicate entity" << endl;
    }
}

void testTraversals() {
    cout << "\n=== Testing BFS and DFS ===" << endl;
    DGraphModel<int> g(intEQ, int2str);
    g.add(1); g.add(2); g.add(3);
    g.connect(1, 2, 1);
    g.connect(1, 3, 1);

    string bfsResult = g.BFS(1);
    cout << "BFS Result: " << bfsResult << endl;
    assertTrue(bfsResult.length() > 5, "BFS returned non-empty string");

    string dfsResult = g.DFS(1);
    cout << "DFS Result: " << dfsResult << endl;
    assertTrue(dfsResult.length() > 5, "DFS returned non-empty string");
}

void testCommonAncestors() {
    cout << "\n=== Testing findCommonAncestors Logic ===" << endl;
    KnowledgeGraph kg;
    
    // Hierarchy: Root -> LeafA, Root -> LeafB
    kg.addEntity("Root");
    kg.addEntity("LeafA");
    kg.addEntity("LeafB");
    kg.addRelation("Root", "LeafA", 5.0);
    kg.addRelation("Root", "LeafB", 5.0);
    
    string ancestor = kg.findCommonAncestors("LeafA", "LeafB");
    assertEqual(ancestor, string("Root"), "Simple common ancestor is Root");
    
    // Disconnected
    kg.addEntity("Alien");
    string noAnc = kg.findCommonAncestors("LeafA", "Alien");
    assertEqual(noAnc, string("No common ancestor"), "Disconnected nodes have no ancestor");
}

// =============================================================================
// Requested Test Case: tc_005
// =============================================================================
void tc_005() {
    cout << "\n=== tc_005: Test toString methods of Edge, VertexNode, and DGraphModel ===" << endl;

    // Define function pointers (lambdas decay to function pointers since no capture)
    bool (*vertexEQ)(string&, string&) = [](string& a, string& b) -> bool { return a == b; };
    string (*vertex2str)(string&) = [](string& s) -> string { return s; };

    DGraphModel<string> graph(vertexEQ, vertex2str);

    // Add vertices
    graph.add("A");
    graph.add("B");
    graph.add("C");

    // Add connections
    graph.connect("A", "B", 2.5f);
    graph.connect("B", "C", 3.0f);
    graph.connect("A", "C", 1.0f);

    // Test DGraphModel toString (which internally uses VertexNode toString and Edge toString)
    cout << "DGraphModel toString:" << endl;
    string graphString = graph.toString();
    cout << graphString << endl;

    // Validate expected substring presence to ensure toString isn't empty or garbage
    assertTrue(graphString.find("(A,") != string::npos, "Graph string contains Node A");
    assertTrue(graphString.find("2.5") != string::npos, "Graph string contains weight 2.5");

    // Test individual VertexNode toString by getting vertex and checking outward edges
    cout << "Testing individual components:" << endl;
    try {
        vector<Edge<string>*> outwardEdges = graph.getOutwardEdges("A");
        cout << "Vertex A has " << outwardEdges.size() << " outward edges" << endl;
        assertEqual((int)outwardEdges.size(), 2, "Vertex A has correct edge count");
        
        if (!outwardEdges.empty()) {
            cout << "Sample Edge toString: " << outwardEdges[0]->toString() << endl;
        }
    } catch (const exception& e) {
        cout << "[FAIL] Exception in tc_005: " << e.what() << endl;
    }
}

// =============================================================================
// Main
// =============================================================================
int main() {
    try {
         testLowLevelGraph();
         testKnowledgeGraphBasics();
         testTraversals();
         testCommonAncestors();
        
        // Run the new test case
        //tc_005();

        cout << "\n=========================================" << endl;
        cout << "Test Summary: " << passedTests << "/" << totalTests << " passed." << endl;
        cout << "=========================================" << endl;
        
    } catch (const exception& e) {
        cout << "CRITICAL FAILURE: Uncaught exception in main: " << e.what() << endl;
    }
    return 0;
}