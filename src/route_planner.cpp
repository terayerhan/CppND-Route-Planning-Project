#include "route_planner.h"
#include <algorithm>
#include <unordered_set>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);

}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);

}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    // Populate the curent_node's neighhors vector
    current_node->FindNeighbors();

    // Loop through all the current_node's neighbors and update them
    for(auto neighbor_ptr : current_node->neighbors) {
        neighbor_ptr->parent = current_node;
        neighbor_ptr->h_value = CalculateHValue(neighbor_ptr);

        // g_value of neighbor = g_value of current node + distance from current node to neighbor node.
        neighbor_ptr->g_value = current_node->g_value + current_node->distance(*neighbor_ptr);
        neighbor_ptr->visited = true; // mark the node as visited. 
        open_list.push_back(neighbor_ptr); // Add the neighbor to the open list.                
    }

}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {
    // Sort the open_list vector RouteModel::Node pointers in descending order based on the sum of their g_value and h_value.
    std::sort(open_list.begin(), open_list.end(), [](auto a, auto b) {
        return (a->g_value + a->h_value) > (b->g_value + b->h_value);
    });

    RouteModel::Node *lowest_cost_node_ptr; // Create a pointer to a Node to store the lowest cost node pointer.
    lowest_cost_node_ptr = open_list.back(); // Get the last node pointer from the vector.
    open_list.pop_back(); // Remove the last Node pointer from the open list.
    
    return lowest_cost_node_ptr;

}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
    path_found.push_back(*current_node); // Push the current node onto the path_found vector.
    RouteModel::Node *parent_node_ptr = current_node->parent; // Get the pointer to the current_node's parent.

    // Check if the parent_node_ptr is not a null pointer since only the start_node will have a null pointer parent.
    while(parent_node_ptr) {
        path_found.push_back(*parent_node_ptr); // push the parent node of the current node onto the path_found vector.
        distance += current_node->distance(*parent_node_ptr); // Add the distance from the current_node to its parent node.
        current_node = parent_node_ptr; // Make the parent_node_ptr the new current_node pointer.
        parent_node_ptr = current_node->parent; // Make the parent_node_ptr point to the parent of the new current_node.
    }

    // Reverse the order of nodes in the path_found vector so that the path start from the start_node to the current_node.
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}



// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.

    //Initialize the current_node coordinates, g, and h values.
    current_node = start_node; // Set the coordinates to those of the start_node.
    current_node->g_value = 0.0f; // Set the g_value to 0 since it is the start_node.
    current_node->h_value = CalculateHValue(current_node); 
    current_node->visited = true; // Mark the start node as visited before inserting into open_list

    // Add the current node to the open list.
    open_list.push_back(current_node);

    // Loop while the open_list is not empty.
    while (!open_list.empty()) { 
        current_node = NextNode(); // get the least costly f_value node.        

        // Check if the curent_node is the goal_node.
        if ((current_node->x == end_node->x) && (current_node->y == end_node->y)) {
            // Construct the final path and store it in m_Model's path attribute.
            m_Model.path = ConstructFinalPath(current_node);            
            return;
        }
        
        // Add current_node's valid neighbors to the open_list.
        AddNeighbors(current_node);
    }  


}