#lang dssl2

# Final project: Trip Planner

import cons
import sbox_hash
import 'project-lib/dictionaries.rkt'
import 'project-lib/graph.rkt'
import 'project-lib/binheap.rkt'
let eight_principles = ["Know your rights.",
"Acknowledge your sources.",
"Protect your work.",
"Avoid suspicion.",
"Do your own work.",
"Never falsify a record or permit another person to do so.",
"Never fabricate data, citations, or experimental results.",
"Always tell the truth when discussing your work with your instructor."]

### Basic Types ###

#  - Latitudes and longitudes are numbers:
let Lat?  = num?
let Lon?  = num?

#  - Point-of-interest categories and names are strings:
let Cat?  = str?
let Name? = str?

### Raw Item Types ###

#  - Raw positions are 2-element vectors with a latitude and a longitude
let RawPos? = TupC[Lat?, Lon?]

#  - Raw road segments are 4-element vectors with the latitude and
#    longitude of their first endpoint, then the latitude and longitude
#    of their second endpoint
let RawSeg? = TupC[Lat?, Lon?, Lat?, Lon?]

#  - Raw points-of-interest are 4-element vectors with a latitude, a
#    longitude, a point-of-interest category, and a name
let RawPOI? = TupC[Lat?, Lon?, Cat?, Name?]

### Contract Helpers ###

# ListC[T] is a list of `T`s (linear time):
let ListC = Cons.ListC
# List of unspecified element type (constant time):
let List? = Cons.list?

struct pos:
    let lat
    let long

struct road:
    let pos1
    let pos2
    let dist
    
struct POI:
    let pos
    let category
    let name
    
interface TRIP_PLANNER:

    # Returns the positions of all the points-of-interest that belong to
    # the given category.
    def locate_all(
            self,
            dst_cat:  Cat?           # point-of-interest category
        )   ->        ListC[RawPos?] # positions of the POIs

    # Returns the shortest route, if any, from the given source position
    # to the point-of-interest with the given name.
    def plan_route(
            self,
            src_lat:  Lat?,          # starting latitude
            src_lon:  Lon?,          # starting longitude
            dst_name: Name?          # name of goal
        )   ->        ListC[RawPos?] # path to goal

    # Finds no more than `n` points-of-interest of the given category
    # nearest to the source position.
    def find_nearby(
            self,
            src_lat:  Lat?,          # starting latitude
            src_lon:  Lon?,          # starting longitude
            dst_cat:  Cat?,          # point-of-interest category
            n:        nat?           # maximum number of results
        )   ->        ListC[RawPOI?] # list of nearby POIs


class TripPlanner (TRIP_PLANNER):
    let pos_to_node_id
    let node_id_to_pos
    let road_data
    let n_roads
    let road_structs
    let poi_data
    let n_pois
    let name_to_poi
    
    
    # constructor that takes in vector of road segments & vector of POIs
    def __init__(self, road_data, poi_data):
        # raw data of roads and POIs
        self.road_data = road_data
        self.poi_data = poi_data
        # length of raw data vectors
        self.n_roads = road_data.len()
        self.n_pois = poi_data.len()
        # create empty vectors to store road structs
        self.road_structs = [ None ; self.road_data.len()]
        # bidirectional hashmap from road positions to node numbers
        self.pos_to_node_id = HashTable(self.n_roads*2, make_sbox_hash())
        self.node_id_to_pos = HashTable(self.n_roads*2, make_sbox_hash())
        # one-way hashmap from POI name to POI struct
        self.name_to_poi = HashTable(self.n_pois, make_sbox_hash())
        # fill bidirectional hashmap from pos to node and node to pos
        self.fill_road_pos_hashes()
        # fill hashmap mapping from POI name to POI struct
        self.fill_poi_hash()
        
        
    # returns all of TripPlanner road structs in form of vector
    def get_road_structs(self):
        for i, r_d in self.road_data:
            let pos1 = pos(r_d[0], r_d[1])
            let pos2 = pos(r_d[2], r_d[3])
            let dist = ((pos2.long - pos1.long)**2 + (pos2.lat - pos1.lat)**2)**(1/2)
            self.road_structs[i] = road(pos1, pos2, dist)
        return self.road_structs
 
        
    def locate_all(self, dst_cat: Cat?) -> ListC[RawPos?]:
        # go through poi_data via for loop, checking to see if a poi in the list has the same category
        # as dst_cat --> add to LL if so
        let list = None
        for poi in self.poi_data:
            
            # obtain relevant info from each POI
            let lat = poi[0]
            let long = poi[1]
            let category = poi[2]
            
            # check to see if current POI has the same category as dst_cat
            if category == dst_cat:
                # loop through current linked list of positions (what we will return) to see if we have this position
                # accounted for already b/c no duplicates allowed
                # ***LATER: possible to do this in O(1) time using a hashtable maybe?
                let node = list
                let duplicate_found = False
                while node:
                    if node.data == [lat, long]:
                        duplicate_found = True
                    node = node.next
                # only 
                if not duplicate_found:
                    let temp = list
                    # add position [lat?, long?] to the linked list we will return
                    list = cons([lat, long], temp)                         
        return list
                
        
        
    def fill_poi_hash(self):
        for i, poi in self.poi_data:
            let pos1 = pos(poi[0], poi[1])
            let cat = poi[2]
            let name = poi[3]
            let poi_struct = POI(pos1, cat, name)
            self.name_to_poi.put(name, poi_struct)
            
            
    def check_pos_hash_mem(self, pos, num):
        if not self.pos_to_node_id.mem?(pos):
            self.pos_to_node_id.put(pos, num)
            self.node_id_to_pos.put(num, pos)
            return False
        return True                    
            
        
    def fill_road_pos_hashes(self):
        # first fill 2-way hashtable to go from road position to node # for graph
        # when filling hashtable, if we get a position we have seen before, it should map to the same node number
        let road_structs = self.get_road_structs()
        let i = 0
        for r_d in road_structs:
            let pos1 = r_d.pos1
            let pos2 = r_d.pos2
            let dist = r_d.dist
            
            # check to see if we have already visited this position
            if not self.check_pos_hash_mem(pos1, i):
                i = i + 1
                
            if not self.check_pos_hash_mem(pos2, i):
                i = i + 1
                
                
                
    def construct_pos_graph(self):
        let road_structs = self.get_road_structs()
        # construct graph with all road endpoints, each node being a vector s.t. [lat, long]
        let road_graph = WUGraph(self.n_roads*2)
        for i, r in road_structs:
            let e1 = self.pos_to_node_id.get(r.pos1)
            let e2 = self.pos_to_node_id.get(r.pos2)
            let dist = r.dist
            road_graph.set_edge(e1, e2, dist)
        return road_graph
            
            
        
    def plan_route(self, src_lat: Lat?, src_lon: Lon?, dst_name: Name?) -> ListC[RawPos?]:
        # first, we need to construct a graph with all roads+endpoints and see where POIs occur
        # then, we need to find the position of the desired POI (dst_name)
        # last, we find a path (if any) from beginning location to dst_name's position (dijkstra)
        
        let road_graph = self.construct_pos_graph()
        let dst_position
        # fill POI hashtable (done in constructor) --> get position of dst_poi
        if self.name_to_poi.mem?(dst_name):
            dst_position = self.name_to_poi.get(dst_name).pos
        else:
            return None
        
        # now we have starting position and dst_position
        # lets implement an algorithm that allows us to get the shortest path = Dijkstra's
        # distances to keep track of shortest distances from nodes, preds to keep track of path, done to keep
        # track of what nodes we have visited, heap is our PQ
        let distances = [ inf ; self.n_roads*2 ]
        let preds = [ None ; self.n_roads*2 ]
        let done = [ False ; self.n_roads*2 ]
        let heap = BinHeap(self.n_roads*2, λ x, y: x[0] < y[0])
        # get node id of starting position --> insert into PQ with distance 0
        let start_vertex = self.pos_to_node_id.get(pos(src_lat, src_lon))
        distances[start_vertex] = 0
        heap.insert(start_vertex)
        let closest_vertex
        # while there are still things to remove from the PQ, pick the nearest vertex
        # get neighbors of said vertex, then add neighbor with least distance to PQ
        while heap.len() > 0:
            # get closest distance and vertex from heap + remove it
            closest_vertex = heap.find_min()
            if vec?(closest_vertex):
                closest_vertex = closest_vertex[1]
            heap.remove_min()
            # continue with while loop if we haven't visited this vertex before
            if not done[closest_vertex]:
                # mark that we have visited this vertex
                done[closest_vertex] = True
                # linked list of neighbors of vertex "closest" in form of: cons(1, cons(2, None))
                let neighbors = road_graph.get_adjacent(closest_vertex)
                # loop through adjacent neighbors of current vertex to see which has the smallest path
                while neighbors:
                    # find current distance (miles) from closest vertex to current neighbor we are looking at
                    let curr_weight = road_graph.get_edge(closest_vertex, neighbors.data)
                    # if distance to current neighbor (weight plus curr distance) is less than what we have stored for this neighbor, then
                    # make that the new distance for the current neighbor
                    if distances[closest_vertex] + curr_weight < distances[neighbors.data]:
                        distances[neighbors.data] = distances[closest_vertex] + curr_weight
                        # mark predecessor of neighbor
                        preds[neighbors.data] = closest_vertex
                        # insert the neighbor into the heap with its vertex number
                        heap.insert([distances[closest_vertex] + curr_weight, neighbors.data])
                    # check next neighbor
                    neighbors = neighbors.next
                    
        # now we use 'preds' to backtrack and record the shortest path
        # we know the most recently visited node is the starting position struct associated with the node #, so
        # we can use that as a starting point to backtrack the shortest path
        let end = self.name_to_poi.get(dst_name)
        let end_coords = [end.pos.lat, end.pos.long]
        end = self.pos_to_node_id.get(end.pos)
        let curr_vertex = end
        let path = None
        while curr_vertex is not None:
            let temp = path
            let data = self.node_id_to_pos.get(curr_vertex)
            let lat = data.lat
            let long = data.long
            if distances[curr_vertex] == inf:
                return None
            path = cons([lat, long], temp)
            curr_vertex = preds[curr_vertex]
        return path

             
        
        
    def find_nearby(self, src_lat: Lat?, src_lon: Lon?, dst_cat: Cat?, n: nat?) -> ListC[RawPOI?]:
        # use shortest path algorithm (dijkstra) to find (up to) the n closest POIs to starting pos with the matching category user
        # wants (aka dst_cat) 
        # we want to start at starting position, then branch out using dijkstra's; when we reach a position that has a POI, then
        # we should add that POI (raw form) to our linked list of POIs (what we will return)
        let num_found = 0
        let road_graph = self.construct_pos_graph()
        
        
        # now we have starting position
        # lets implement an algorithm that allows us to get the shortest path = Dijkstra's
        # distances to keep track of shortest distances from nodes, preds to keep track of path, done to keep
        # track of what nodes we have visited, heap is our PQ
        let distances = [ inf ; self.n_roads*2 ]
        let preds = [ None ; self.n_roads*2 ]
        let done = [ False ; self.n_roads*2 ]
        let heap = BinHeap(self.n_roads*2, λ x, y: x[0] < y[0])
        # get node id of starting position --> insert into PQ with distance 0
        let start_vertex
        if self.pos_to_node_id.mem?(pos(src_lat, src_lon)):
            start_vertex = self.pos_to_node_id.get(pos(src_lat, src_lon))
        else:
            return None
        distances[start_vertex] = 0
        heap.insert(start_vertex)
        let closest_vertex
        let list_pois = None
        
        while heap.len() > 0 and num_found < n:
            # find closest node + remove it from heap
            closest_vertex = heap.find_min()
            if vec?(closest_vertex):
                closest_vertex = closest_vertex[1]
            heap.remove_min()
            # check to see if this position has a POI 
            for poi in self.poi_data:
               let poi_struct = self.name_to_poi.get(poi[3])
               if poi_struct.category == dst_cat:
                   let coords_struct = poi_struct.pos
                   let poi_node_id = self.pos_to_node_id.get(pos(coords_struct.lat, coords_struct.long))
                   if poi_node_id == closest_vertex:
                       let temp = list_pois
                       list_pois = cons(poi, temp)
                       num_found = num_found + 1
                       if num_found >= n:
                           break
                   
        # continue with while loop if we haven't visited this vertex before
            if not done[closest_vertex]:
                # mark that we have visited this vertex
                done[closest_vertex] = True
                # linked list of neighbors of vertex "closest" in form of: cons(1, cons(2, None))
                let neighbors = road_graph.get_adjacent(closest_vertex)
                
                # loop through adjacent neighbors of current vertex to see which has the smallest path
                while neighbors:
                    # find current distance (miles) from closest vertex to current neighbor we are looking at
                    let curr_weight = road_graph.get_edge(closest_vertex, neighbors.data)
                    # if distance to current neighbor (weight plus curr distance) is less than what we have stored for this neighbor, then
                    # make that the new distance for the current neighbor
                    if distances[closest_vertex] + curr_weight < distances[neighbors.data]:
                        distances[neighbors.data] = distances[closest_vertex] + curr_weight
                        # mark predecessor of neighbor
                        preds[neighbors.data] = closest_vertex
                        # insert the neighbor into the heap with its vertex number
                        heap.insert([distances[closest_vertex] + curr_weight, neighbors.data])
         
                    # check next neighbor
                    neighbors = neighbors.next 
        return list_pois 


def my_first_example():
    return TripPlanner([[0,0, 0,1], [0,0, 1,0]],
                       [[0,0, "bar", "The Empty Bottle"],
                        [0,1, "food", "Pierogi"]])
                        
                   
test 'My first locate_all test':
    assert my_first_example().locate_all("food") == \
        cons([0,1], None)
    let trip = TripPlanner([[0,0, 0,1], [0,0, 1,0]],
                       [[0,0, "bar", "The Empty Bottle"],
                        [0,1, "food", "Pierogi"],
                        [0,3, "food", "Chipotle"],
                        [0,0, "food", "Meow-sicle"],
                        [0,0, "food", "Beef Wellington"]])
    assert trip.locate_all("food") == cons([0, 0], cons([0, 3], cons([0, 1], None)))  
    
    let trip2 = TripPlanner([[0,0, 1,-1], [1,-1, 1,-1], [0,0, -1,-3], [-2,2, -1,4], [-1,4, 1,2], [1,2, 2,4], [2,4, 2,4], [2,4, 4,1], [4,1, 2,1], [0,0, 1,2], [0,0, 0,0], [-1,4, 0,0]],
                            [[-1,4, "bar", "The Clam"],
                            [2,4, "food", "Chipotle"],
                            [2,1, "food", "Burger P"],
                            [0,0, "salon", "Grace's"],
                            [1,2, "theater", "AMC"],
                            [1,-1, "food", "Lisa's"],
                            [-1,-3, "bar", "Reza's"]])
    assert trip2.locate_all("food") == cons([1,-1], cons([2,1], cons([2,4], None)))
    assert trip2.locate_all("bar") == cons([-1,-3], cons([-1,4], None))
    assert trip2.locate_all("salon") == cons([0,0], None)
    assert trip2.locate_all('theater') == cons([1,2], None)                           

    
    
test 'My first plan_route test':
   assert my_first_example().plan_route(0, 0, "Pierogi") == \
       cons([0,0], cons([0,1], None))
   let trip2 = TripPlanner([
                [0, 0, 1, 1],
                [1, 1, 2, 2],
                [2, 2, 3, 3],
                [3, 3, 4, 4],
                [4, 4, 5, 5],
                [2, 2, 1, 4],
                [1, 1, 1, 4],
                [3, 3, 2, 5],
                [1, 4, 2, 5],
                [2, 2, 4, 2]],
                       [[0,0, "bar", "The Empty Bottle"],
                        [1,4, "food", "Pierogi"],
                        [4,4, "food", "Chipotle"],
                        [4, 2, "games", "GameStop"],
                        [2, 2, "food", "yasss"],
                        [2, 5, "food", "Lisas"]])
                        
   assert trip2.plan_route(0, 0, "Lisas") == cons([0,0], cons([1,1], cons([1, 4], cons([2, 5], None))))
   
   let trip3 = TripPlanner([[0,0, 1,-1], [1,-1, 1,-1], [0,0, -1,-3], [-2,2, -1,4], [-1,4, 1,2], [1,2, 2,4], [2,4, 2,4], [2,4, 4,1], [4,1, 2,1], [0,0, 1,2], [0,0, 0,0], [-1,4, 0,0]],
                            [[-1,4, "bar", "The Clam"],
                            [2,4, "food", "Chipotle"],
                            [2,1, "food", "Burger P"],
                            [0,0, "salon", "Grace's"],
                            [1,2, "theater", "AMC"],
                            [1,-1, "food", "Lisa's"],
                            [-1,-3, "bar", "Reza's"]])                       
   assert trip3.plan_route(-2, 2, "Chipotle") == cons([-2,2], cons([-1,4], cons([1,2], cons([2,4], None))))
   
   
   
test 'My first find_nearby test':
    assert my_first_example().find_nearby(0, 0, "food", 1) == \
        cons([0,1, "food", "Pierogi"], None)
    let trip2 = TripPlanner([
                [0, 0, 1, 1],
                [1, 1, 2, 2],
                [2, 2, 3, 3],
                [3, 3, 4, 4],
                [4, 4, 5, 5],
                [2, 2, 1, 4],
                [1, 1, 1, 4],
                [3, 3, 2, 5],
                [1, 4, 2, 5],
                [2, 2, 4, 2]],
                       [[0,0, "bar", "The Empty Bottle"],
                        [1,4, "food", "Pierogi"],
                        [4,4, "food", "Chipotle"],
                        [4, 2, "games", "GameStop"],
                        [2, 2, "food", "yasss"],
                        [2, 5, "food", "Lisas"]])
                         
    let list_poi1 = trip2.find_nearby(0,0, "food", 3)
    let locations = [ None ; 3]
    for i in range(3):
        locations[i] = list_poi1.data[3]
        list_poi1 = list_poi1.next
    assert locations == ['Chipotle', 'Pierogi', 'yasss']   
    
    
    
    
test 'basic':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [4, 0, 'food', 'Jollibee'],
       [5, 0, 'barber', 'Judy']])
    let result = tp.find_nearby(0, 0, 'barber', 2)
    #result = _sort_for_grading_comparison(_Cons_to_vec(result))
    assert Cons.to_vec(result) == [[3, 0, 'barber', 'Tony']]    
    
    
test 'advanced route':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony']])
    let result = tp.plan_route(0, 0, 'Judy')
    result = Cons.to_vec(result)
    assert result == []      
    
    let tp2 = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [5, 0, 'barber', 'Judy']])
    let result2 = tp2.plan_route(0, 0, 'Judy')
    result2 = Cons.to_vec(result2)
    assert result2 == []      
    
    
test 'advanced nearby':
    let tp = TripPlanner(
      [[0, 0, 0, 9],
       [0, 9, 9, 9],
       [0, 0, 1, 1],
       [1, 1, 2, 2],
       [2, 2, 3, 3],
       [3, 3, 4, 4],
       [4, 4, 5, 5],
       [5, 5, 6, 6],
       [6, 6, 7, 7],
       [7, 7, 8, 8],
       [8, 8, 9, 9]],
      [[7, 7, 'haberdasher', 'Archit'],
       [8, 8, 'haberdasher', 'Braden'],
       [9, 9, 'haberdasher', 'Cem']])
    let result = tp.find_nearby(0, 0, 'haberdasher', 2)
    #result = _sort_for_grading_comparison(_Cons_to_vec(result))
    result = Cons.to_vec(result)
    for i in range(result.len()):
        if i == result.len()-1:
            break
        if result[i][0] > result[i+1][0]:
            let temp = result[i]
            result[i] = result[i+1]
            result[i+1] = temp
    assert result == [[7, 7, 'haberdasher', 'Archit'], [8, 8, 'haberdasher', 'Braden']]    
    
    let tp2 = TripPlanner(
      [[-1.1, -1.1, 0, 0],
       [0, 0, 3, 0],
       [3, 0, 3, 3],
       [3, 3, 3, 4],
       [0, 0, 3, 4]],
      [[0, 0, 'food', 'Sandwiches'],
       [3, 0, 'bank', 'Union'],
       [3, 3, 'barber', 'Judy'],
       [3, 4, 'barber', 'Tony']])
    let result2 = tp2.find_nearby(-1.1, -1.1, 'barber', 1)
    #result22 = _sort_for_grading_comparison(_Cons_to_vec(result))
    assert Cons.to_vec(result2) == [[3, 4, 'barber', 'Tony']]
    
    
    let tp3 = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [4, 0, 'food', 'Jollibee'],
       [5, 0, 'barber', 'Judy']])
    let result3 = tp.find_nearby(0, 0, 'food', 1)
    #result3 = _sort_for_grading_comparison(_Cons_to_vec(result))
    assert Cons.to_vec(result3) == []