#include <iostream>
#include <vector>
#include <algorithm>
#include <numeric>
#include <cassert>

using namespace std;

// --- Segment Tree (for sum queries and point updates) ---
class SegmentTree {
public:
    /**
     * @brief Constructs a new Segment Tree object.
     * 
     * @param size The size of the array the segment tree will represent.
     *
     * @note Space complexity: O(size) for storing the tree (typically 4*size).
     */
    SegmentTree(int size) : n(size) {
        tree.resize(4 * n, 0);
    }

    /**
     * @brief Builds the segment tree from a vector of values already mapped to segment tree positions.
     * 
     * @param values_at_pos The vector containing the values at their corresponding positions in the flattened tree.
     *
     * @note Time complexity: O(size), where size is the size of the segment tree (N nodes).
     */
    void build_from_mapped_values(const vector<int>& values_at_pos) {
        if (values_at_pos.empty()) {
            return; 
        }
        build(values_at_pos, 0, 0, n - 1);
    }

    /**
     * @brief Updates the value at a specific index in the segment tree.
     * 
     * @param index The index to update (in the original array's mapping).
     * @param value The new value for the index.
     *
     * @note Time complexity: O(log size), where size is the size of the segment tree (N nodes).
     */
    void update(int index, int value) {
        update(0, 0, n - 1, index, value);
    }

    /**
     * @brief Queries the sum of values in a given range [query_left, query_right].
     * 
     * @param query_left The starting index of the query range.
     * @param query_right The ending index of the query range.
     * @return The sum of values in the specified range.
     *
     * @note Time complexity: O(log size), where size is the size of the segment tree (N nodes).
     */
    int query(int query_left, int query_right) {
        if (query_left > query_right) return 0;
        return query(0, 0, n - 1, query_left, query_right);
    }

private:
    int n; // Size of the original array/flattened tree array
    vector<int> tree; // Stores the segment tree nodes

    /**
     * @brief Combines the results of two segment tree nodes.
     *        For this sum segment tree, it simply adds the values.
     * 
     * @param a Value from the left child node.
     * @param b Value from the right child node.
     * @return The combined value (sum).
     */
    int combine(int a, int b) {
        return a + b;
    }

    /**
     * @brief Recursive helper function to build the segment tree.
     * 
     * @param arr The input array/vector from which to build the tree.
     * @param node The current node index in the segment tree array.
     * @param start The starting index of the current segment.
     * @param end The ending index of the current segment.
     */
    void build(const vector<int>& arr, int node, int start, int end) {
        if (start == end) {
            tree[node] = arr[start];
        } else {
            int mid = (start + end) / 2;
            build(arr, 2 * node + 1, start, mid);
            build(arr, 2 * node + 2, mid + 1, end);
            tree[node] = combine(tree[2 * node + 1], tree[2 * node + 2]);
        }
    }

    /**
     * @brief Recursive helper function to update a value at a specific index.
     * 
     * @param node The current node index in the segment tree array.
     * @param start The starting index of the current segment.
     * @param end The ending index of the current segment.
     * @param idx The index to update in the original array's mapping.
     * @param val The new value for the index.
     */
    void update(int node, int start, int end, int idx, int val) {
        if (start == end) {
            tree[node] = val;
        } else {
            int mid = (start + end) / 2;
            if (start <= idx && idx <= mid) {
                update(2 * node + 1, start, mid, idx, val);
            } else {
                update(2 * node + 2, mid + 1, end, idx, val);
            }
            tree[node] = combine(tree[2 * node + 1], tree[2 * node + 2]);
        }
    }

    /**
     * @brief Recursive helper function to query the sum in a range [l, r].
     * 
     * @param node The current node index in the segment tree array.
     * @param start The starting index of the current segment.
     * @param end The ending index of the current segment.
     * @param l The left boundary of the query range.
     * @param r The right boundary of the query range.
     * @return The sum of values in the specified range.
     */
    int query(int node, int start, int end, int l, int r) {
        if (r < start || end < l) {
            return 0;
        }
        if (l <= start && end <= r) {
            return tree[node];
        }
        int mid = (start + end) / 2;
        int p1 = query(2 * node + 1, start, mid, l, r);
        int p2 = query(2 * node + 2, mid + 1, end, l, r);
        return combine(p1, p2);
    }
};

// --- Heavy-Light Decomposition Class ---
class HLD {
public:
    /**
     * @brief Constructs a new HLD object for a given tree.
     * 
     * @param num_nodes The total number of nodes in the tree (0-indexed).
     * @param node_initial_values A vector containing the initial values for each node.
     */
    HLD(int num_nodes, const vector<int>& node_initial_values)
        : N(num_nodes),
          adj(num_nodes),
          values(node_initial_values),
          parent(num_nodes, -1),
          depth(num_nodes, 0),
          subtree_size(num_nodes, 1),
          heavy_child(num_nodes, -1),
          head(num_nodes),
          pos(num_nodes),
          cur_pos(0),
          seg_tree(num_nodes) {
    }

    /**
     * @brief Adds an edge between two nodes in the tree.
     *        Assumes an undirected tree structure.
     * 
     * @param u The first node.
     * @param v The second node.
     */
    void add_edge(int u, int v) {
        adj[u].push_back(v);
        adj[v].push_back(u);
    }

    /**
     * @brief Builds the Heavy-Light Decomposition structure and the underlying segment tree.
     *        Call this after adding all edges.
     * @param root The root node of the tree.
     * @note Time complexity: O(N) for DFS1 and DFS2 + O(N) for segment tree build = O(N)
     * @note Space complexity: O(N) for various vectors and the segment tree
     */
    void build(int root) {
        dfs1_size_depth_parent(root, -1, 0);
        dfs2_hld(root, root);

        vector<int> values_for_seg_tree(N);
        for (int i = 0; i < N; ++i) {
            values_for_seg_tree[pos[i]] = values[i];
        }
        seg_tree.build_from_mapped_values(values_for_seg_tree);
    }

    /**
     * @brief Updates the value of a specific node and propagates the change to the segment tree.
     * 
     * @param u The node whose value needs to be updated.
     * @param new_value The new value for node u.
     *
     * @note Time complexity: O(log N) due to segment tree update.
     */
    void update_node_value(int u, int new_value) {
        values[u] = new_value;
        seg_tree.update(pos[u], new_value);
    }

    /**
     * @brief Queries the sum of values on the path between two nodes.
     * @param u The first node.
     * @param v The second node.
     * @return The sum of values on the path between u and v.
     *
     * @note Time complexity: O(log^2 N) in the worst case (path crossing many heavy paths).
     */
    int query_path(int u, int v) {
        int result = 0;

        while (head[u] != head[v]) {
            if (depth[head[u]] < depth[head[v]]) {
                swap(u, v);
            }
            result += seg_tree.query(pos[head[u]], pos[u]);
            u = parent[head[u]];
        }

        if (depth[u] > depth[v]) {
            swap(u, v);
        }
        result += seg_tree.query(pos[u], pos[v]);
        
        return result;
    }

    /**
     * @brief Finds the Lowest Common Ancestor (LCA) of two nodes.
     * @param u The first node.
     * @param v The second node.
     * @return The index of the LCA node.
     *
     * @note Time complexity: O(log N).
     */
    int get_lca(int u, int v) {
        while (head[u] != head[v]) {
            if (depth[head[u]] < depth[head[v]]) {
                swap(u, v);
            }
            u = parent[head[u]];
        }
        return (depth[u] < depth[v]) ? u : v;
    }


private:
    int N; // Total number of nodes in the tree
    vector<vector<int>> adj; // Adjacency list for the tree
    vector<int> values; // Stores original values at nodes

    vector<int> parent;      // Stores the parent of each node in the DFS tree
    vector<int> depth;       // Stores the depth of each node (distance from root)
    vector<int> subtree_size; // Stores the size of the subtree rooted at each node
    vector<int> heavy_child; // Stores the heavy child of a node, -1 if none
    vector<int> head;        // Stores the head of the heavy path node u belongs to
    vector<int> pos;         // Stores the position of node u in the flattened segment tree array
    int cur_pos;                  // Current position counter for the segment tree array

    SegmentTree seg_tree; // Segment tree to store values on flattened heavy paths

    /**
     * @brief First DFS pass to calculate subtree sizes, depths, and parents,
     *        and identify the heavy child for each node.
     *
     * @param u The current node being visited.
     * @param p The parent of the current node.
     * @param d The depth of the current node from the root.
     */
    void dfs1_size_depth_parent(int u, int p, int d) {
        parent[u] = p;
        depth[u] = d;
        subtree_size[u] = 1;
        int max_c_subtree_size = 0;

        for (int v : adj[u]) {
            if (v == p) continue;
            dfs1_size_depth_parent(v, u, d + 1);
            subtree_size[u] += subtree_size[v];
            if (subtree_size[v] > max_c_subtree_size) {
                max_c_subtree_size = subtree_size[v];
                heavy_child[u] = v;
            }
        }
    }

    /**
     * @brief Second DFS pass to perform Heavy-Light Decomposition.
     *        Assigns chain heads and positions in the flattened array.
     *
     * @param u The current node being visited.
     * @param h The head of the heavy path the current node belongs to.
     */
    void dfs2_hld(int u, int h) {
        head[u] = h;
        pos[u] = cur_pos++;

        if (heavy_child[u] != -1) {
            dfs2_hld(heavy_child[u], h);
        }

        for (int v : adj[u]) {
            if (v == parent[u] || v == heavy_child[u]) continue;
            dfs2_hld(v, v);
        }
    }
};

void test_single_node_tree() {
    cout << "Running test_single_node_tree..." << endl;
    vector<int> node_values = {100};
    HLD hld_solver(1, node_values);
    hld_solver.build(0);

    assert(hld_solver.query_path(0, 0) == 100);
    assert(hld_solver.get_lca(0, 0) == 0);

    hld_solver.update_node_value(0, 50);
    assert(hld_solver.query_path(0, 0) == 50);
    cout << "test_single_node_tree PASSED" << endl;
}

void test_line_graph() {
    cout << "Running test_line_graph..." << endl;
    int n = 4;
    vector<int> node_values = {10, 20, 30, 40};
    HLD hld_solver(n, node_values);
    hld_solver.add_edge(0, 1);
    hld_solver.add_edge(1, 2);
    hld_solver.add_edge(2, 3);
    hld_solver.build(0);

    assert(hld_solver.query_path(0, 0) == 10);
    assert(hld_solver.query_path(1, 1) == 20);
    assert(hld_solver.query_path(0, 1) == 10 + 20);
    assert(hld_solver.query_path(1, 0) == 10 + 20);
    assert(hld_solver.query_path(0, 3) == 10 + 20 + 30 + 40);
    assert(hld_solver.query_path(3, 0) == 10 + 20 + 30 + 40);
    assert(hld_solver.query_path(1, 2) == 20 + 30);
    assert(hld_solver.query_path(2, 3) == 30 + 40);

    assert(hld_solver.get_lca(0, 0) == 0);
    assert(hld_solver.get_lca(0, 3) == 0);
    assert(hld_solver.get_lca(1, 3) == 1);
    assert(hld_solver.get_lca(2, 3) == 2);
    assert(hld_solver.get_lca(3, 1) == 1);

    hld_solver.update_node_value(1, 200);
    assert(hld_solver.query_path(0, 0) == 10);
    assert(hld_solver.query_path(1, 1) == 200);
    assert(hld_solver.query_path(0, 1) == 10 + 200);
    assert(hld_solver.query_path(0, 3) == 10 + 200 + 30 + 40);
    cout << "test_line_graph PASSED" << endl;
}

void test_star_graph() {
    cout << "Running test_star_graph..." << endl;
    int n = 4;
    vector<int> node_values = {100, 10, 20, 30};
    HLD hld_solver(n, node_values);
    hld_solver.add_edge(0, 1);
    hld_solver.add_edge(0, 2);
    hld_solver.add_edge(0, 3);
    hld_solver.build(0);

    assert(hld_solver.query_path(0, 0) == 100);
    assert(hld_solver.query_path(1, 1) == 10);
    assert(hld_solver.query_path(0, 1) == 100 + 10);
    assert(hld_solver.query_path(1, 2) == 10 + 100 + 20);
    assert(hld_solver.query_path(2, 1) == 10 + 100 + 20);
    assert(hld_solver.query_path(1, 3) == 10 + 100 + 30);

    assert(hld_solver.get_lca(0, 1) == 0);
    assert(hld_solver.get_lca(1, 2) == 0);
    assert(hld_solver.get_lca(1, 0) == 0);
    assert(hld_solver.get_lca(3, 2) == 0);

    hld_solver.update_node_value(0, 5);
    assert(hld_solver.query_path(0, 1) == 5 + 10);
    assert(hld_solver.query_path(1, 2) == 10 + 5 + 20);

    hld_solver.update_node_value(2, 200);
    assert(hld_solver.query_path(1, 2) == 10 + 5 + 200);
    assert(hld_solver.query_path(0, 2) == 5 + 200);
    cout << "test_star_graph PASSED" << endl;
}

void test_original_example_tree() {
    cout << "Running test_original_example_tree..." << endl;
    int n = 7;
    vector<int> node_values = {2, 10, 5, 3, 8, 1, 7};
    HLD hld_solver(n, node_values);
    hld_solver.add_edge(1, 0);
    hld_solver.add_edge(1, 2);
    hld_solver.add_edge(1, 3);
    hld_solver.add_edge(0, 4);
    hld_solver.add_edge(3, 5);
    hld_solver.add_edge(5, 6);
    hld_solver.build(1);

    assert(hld_solver.query_path(4, 6) == (8 + 2 + 10 + 3 + 1 + 7));
    assert(hld_solver.query_path(6, 4) == (8 + 2 + 10 + 3 + 1 + 7));
    assert(hld_solver.query_path(0, 2) == (2 + 10 + 5));
    assert(hld_solver.query_path(1, 1) == 10);
    assert(hld_solver.query_path(6, 6) == 7);
    assert(hld_solver.query_path(1, 6) == (10 + 3 + 1 + 7));

    assert(hld_solver.get_lca(4, 6) == 1);
    assert(hld_solver.get_lca(4, 0) == 0);
    assert(hld_solver.get_lca(2, 5) == 1);
    assert(hld_solver.get_lca(6, 3) == 3);
    assert(hld_solver.get_lca(4, 2) == 1);

    hld_solver.update_node_value(1, 100);
    assert(hld_solver.query_path(4, 6) == (8 + 2 + 100 + 3 + 1 + 7));
    assert(hld_solver.query_path(0, 2) == (2 + 100 + 5));
    
    hld_solver.update_node_value(6, 70);
    assert(hld_solver.query_path(4, 6) == (8 + 2 + 100 + 3 + 1 + 70));
    cout << "test_original_example_tree PASSED" << endl;
}

void run_all_hld_tests() {
    cout << "--- Starting HLD Tests ---" << endl;
    test_single_node_tree();
    test_line_graph();
    test_star_graph();
    test_original_example_tree();
    cout << "--- All HLD Tests Completed ---" << endl;
}

void run_hld_sample() {
    cout << "\n--- Running HLD Sample ---" << endl;
    int n = 7;
    vector<int> node_values = {2, 10, 5, 3, 8, 1, 7};
    HLD hld_solver(n, node_values);
    hld_solver.add_edge(1, 0);
    hld_solver.add_edge(1, 2);
    hld_solver.add_edge(1, 3);
    hld_solver.add_edge(0, 4);
    hld_solver.add_edge(3, 5);
    hld_solver.add_edge(5, 6);
    hld_solver.build(1);

    cout << "Path sum (4 to 6): " << hld_solver.query_path(4, 6) << endl;
    cout << "Path sum (0 to 2): " << hld_solver.query_path(0, 2) << endl;
    cout << "Path sum (1 to 1): " << hld_solver.query_path(1, 1) << endl;

    cout << "Updating node 1 value from 10 to 100" << endl;
    hld_solver.update_node_value(1, 100);

    cout << "Path sum (4 to 6) after update: " << hld_solver.query_path(4, 6) << endl;
    cout << "Path sum (0 to 2) after update: " << hld_solver.query_path(0, 2) << endl;

    cout << "LCA(4, 6): " << hld_solver.get_lca(4,6) << endl;
    cout << "LCA(4, 0): " << hld_solver.get_lca(4,0) << endl;
    cout << "LCA(2, 5): " << hld_solver.get_lca(2,5) << endl;
    cout << "--- HLD Sample Completed ---" << endl;
}

int main() {
    run_all_hld_tests();
    run_hld_sample();

    return 0;
}