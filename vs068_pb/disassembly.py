#import sys
import os
import vs068_pb.config as config

# OUT_TO_FILE = True

# if OUT_TO_FILE:
#     sys.stdout = open(os.path.join(config.src_fldr, "dissassembly_log.txt"), 'w') # Change to 'a' for append

#     import atexit
#     from datetime import datetime

#     load_time = datetime.now()
#     dt_string = load_time.strftime("%d/%m/%Y %H:%M:%S")
#     print("Logfile time: ", dt_string)

#     @atexit.register
#     def clean_stdout():
#         # Ensure we clean up the logfile when the module is unloaded
#         close_time = datetime.now()
#         dt_string = close_time.strftime("%d/%m/%Y %H:%M:%S")
#         print("Closing logfile at time: ", dt_string)
#         sys.stdout.close()    

import pickle
import numpy as np
import time
import igraph as ig

class Assembly(object):
    def __init__(self, free_directions=None, test_directions=None, liaisons=None, centroids=None, base_ids=[]):
        self.pickle_file = os.path.join(config.src_fldr, "pickles", "assembly.pickle")
        self.tree_pickle = os.path.join(config.src_fldr, "pickles", "assembly_tree.pickle")

        if free_directions == None or test_directions == None or liaisons == None or centroids == None:
            print("Loading directions...")
            self.load_assembly()
        else:
            print("Initialising with new directions...")

            # Convert to tuples for easing set operations
            print(free_directions)
            self.free_directions = [[[tuple(direction) for direction in face] for face in element] for element in free_directions]
            self.test_directions = [tuple(direction) for direction in test_directions]
            self.liaisons = liaisons
            self.centroids = centroids
            self.base = set(base_ids)

            self.save_assembly()


        
        #self.base = set([0, 23, 24, 47, 48, 71, 72, 95])
        self.num_members = len(self.free_directions)
        self.reset_assembly()
        self.build_precedence()
        
        print("Initialised successfully")

    """ 
    General assembly functions 
    """

    def reset_assembly(self):
        self.current_assembly = list(range(self.num_members))
        self.liaison_matrix = [[0 for i in range(self.num_members)] for i in range(self.num_members)]
        
        for e_cnt, element in enumerate(self.liaisons):
            for match in element:
                self.liaison_matrix[e_cnt][match] = 1

        self.combine_all()

    def remove_element(self, num):
        if num in self.current_assembly:
            self.current_assembly.remove(self.current_assembly[self.current_assembly.index(num)])
        else:
            print(num)
            print(self.current_assembly)
            print("Element {} not in assembly".format(num))
    
    def add_element(self, num):
        if num not in self.current_assembly:
            self.current_assembly.append(num)
            self.current_asssembly.sort()
        else:
            print("Element {} already in assembly".format(num))

    def free_to_blocking(self, free):
        blocked = []
        for f in self.test_directions:
            if f not in free:
                blocked.append(f)
        return blocked

    def intersect_lists(self, lists):
        def Intersection(lst1, lst2):
            #print(lst1)
            s1 = set(lst1)
            s2 = set(lst2)
            
            return list(s1 & s2)

        intersection_lists = lists[0]

        for cnt in range(1, len(lists)):
            #print(cnt)
            intersection_lists = Intersection(intersection_lists, lists[cnt])
        return intersection_lists

    def get_full_neighbours(self, element_num):
        return self.liaisons[element_num]

    def get_current_neighbours(self, element_num):
        liaisons_for_element = self.liaisons[element_num]
        return list(set(liaisons_for_element) & set(self.current_assembly))

    def build_precedence(self):
        """ 
        Build lists of precedence and succession relationships (basic, based on picking the lower neighbours). 
        Reset the assembly first!
        """

        self.precedence = []
        for el in range(self.num_members):
            own_z = self.centroids[el][2]

            if el in self.base:
                self.precedence.append([])
            else:
                neighbours = self.get_full_neighbours(el)
                precedence_el = [p for p in neighbours if self.centroids[p][2] < own_z-0.2]
                self.precedence.append(precedence_el)
                # if len(precedence_el) > 0:

                #     self.precedence.append([precedence_el[0]])
                # else:
                #     self.precedence.append(precedence_el)
                #continue # Comment this out for multiple predecessors
        
        self.succession = [[] for i in range(self.num_members)]

        for cnt, el in enumerate(self.precedence):
            for precedent in el:
                self.succession[precedent].append(cnt)
                

    def combine_all(self, blocked=True):
        self.current_frees = [self.combine_ndfgs(i) for i in self.current_assembly]
        self.free_dict = {i : self.combine_ndfgs(i) for i in self.current_assembly}
        if blocked:
            self.current_blocked = [self.free_to_blocking(i) for i in self.current_frees]

    def combine_ndfgs(self, element_num):
        free_lists = []

        liaisons_for_element = self.get_full_neighbours(element_num)
        faces_active = [liaisons_for_element.index(i) for i in self.get_current_neighbours(element_num)]

        if len(faces_active) > 1:
            for face in faces_active: 
                face_curr = self.free_directions[element_num][face]
                free_lists.append(set(face_curr))
        elif faces_active == []:
            # Shouldn't really happen
            return self.test_directions
        else:
            return self.free_directions[element_num][faces_active[0]]
        return self.intersect_lists(free_lists)


    """
    Valid disassembly finding
    """

    def list_retrace(self, node):
        path_lst = []
        for n in node.retrace():
            path_lst.append(n.id_e)
        #print(path_lst)
        return path_lst

    def order_by_freedom(self, min_freedom=3):
        free_elements = []
        free_dir_lens = []
        
        assert len(self.current_frees)==len(self.current_assembly)
        for i in range(len(self.current_assembly)):
            if len(self.current_frees[i]) > min_freedom:
                free_elements.append(self.current_assembly[i])
                free_dir_lens.append(len(self.current_frees[i])) 
        #print(free_elements)
        zipped_lists = zip(free_dir_lens, free_elements)
        sorted_zipped_lists = sorted(zipped_lists, reverse=True)
        #print(sorted_zipped_lists)
        free_elements = [element for _, element in sorted_zipped_lists]
        liaison_list = [self.liaisons[el] for el in free_elements]
        
        return free_elements

    def check_succession(self, element, output=False):
        """ 
        Check to stop us from "pulling the rug out" from under an element in disassembly
        """
        num_ngbrs = len(self.succession[element])

        num_successors = 0
        for ngbr in self.succession[element]:       
            if ngbr in self.current_assembly and ngbr not in self.base:
                num_successors += 1
        if num_successors == num_ngbrs:
            return False
        else:
            return True

    def recursive_disassembler(self, state=None, base=None, fixed_base=True, successful=[], min_freedom=0, depth_mult=5):
        if state == None:
            state = self.save_state()            

        if base == None:
            base = self.TreeNode()
        
        if successful == []:
            successful = []
        # else:
        #     print(successful)
         
        free_elements = self.order_by_freedom(min_freedom=min_freedom)
        if fixed_base:
            free_elements = [i for i in free_elements if i not in self.base]
        
        #free_elements = [f for f in free_elements if self.check_succession(f)]
        #if len(free_elements) > 2:
        #    free_elements = free_elements[:2]

        if len(free_elements) == 0:
            #print("No free elements here! Back up a level to element {}".format(base.id_e))
            return
       
        num_left = len(self.current_assembly)
        if fixed_base:
            num_left -= len(self.base)
        #print("{} left".format(num_left))
        
        i = 0
        i_max = len(free_elements)
        while (True):
            if i >= i_max:
                #print("No MORE free elements! Back up a level to element {}".format(base.id_e))
                return

            element = free_elements[i]
            if base.id_e == -1:
                print("Element num {} = {}".format(i, element))
            i += 1

            self.load_state(state)

            cum_freedom = base.cum_freedom + len(self.current_frees[self.current_assembly.index(element)])
            #print(cum_freedom)
            self.remove_element(element)
            self.combine_all(blocked=False)
            num_left = len(self.current_assembly)
            if fixed_base: 
                num_left -= len(self.base)

            new_node = self.TreeNode(id_e=element, parent=base, num_left=num_left, cum_freedom=cum_freedom)


            if num_left == 0:
                #print("Success!")
                #print(new_node)
                #successful.append(new_node)
                successful += [new_node]
                new_node.set_success()
                if len(successful) % 100 == 0:
                    print("{} successes".format(len(successful)))
                yield successful
                return
            else:
                current_state = self.save_state()

                gen_diss = self.recursive_disassembler(state=current_state, base = new_node, fixed_base=fixed_base, successful=successful, min_freedom=min_freedom, depth_mult=depth_mult)

                n_recurse = 0
                for successes in gen_diss:
                    n_recurse += 1
                    #print("Here")

                    if n_recurse > depth_mult*num_left:
                        break
                    if len(successes) != len(successful):
                        successful = successes
                    #time.sleep(0.001) # May be able to remove now. High RAM usage was caused by addind nodes indiscriminately
                    yield successful 

        #yield successful
        return 

    def disassembly_tree(self, time_limit = 10, min_freedom=0, depth_mult=5):
        self.reset_assembly()
        start_time = time.time()
        end_time = time_limit
        gen_diss = self.recursive_disassembler(min_freedom=min_freedom, depth_mult=depth_mult)
        elements = []
        while time.time() - start_time < end_time:
            try:
                elements = next(gen_diss)
            except StopIteration:
                print("We must have found all disassemblies! Saving")
                print("Time taken : {} seconds".format(int(time.time() - start_time)))
                break

        if time.time() - start_time > end_time:
            print("Timed out at {} seconds".format(int(time.time() - start_time)))

        print("{} possible disassemblies. Saving...".format(len(elements)))
        self.save_to_pickle(elements, self.tree_pickle)
        for i in elements:
            #config_retrace(elements[i])
            if len(self.list_retrace(i)) != len(self.list_retrace(elements[0])):
                self.list_retrace(elements[i])

    def disassemble_loosest(self, fixed_base=True):
        """ 
        Greedy disassembler 
        Assumes that a disassembly can be found by always removing the loosest element
        NEED to add a requirement for a liaison (i.e. to leave others still attached). This should hopefully force it to progagate out/from the ends
        See B+B review (Endnote: Search liaisons) for info on precedence
        """
        partial_liaison_matrix = self.liaison_matrix

        print("Running disassembler")
        directions = []
        elements = []
        cum_freedom = 0

        while( (len(self.current_assembly) != 0 and fixed_base==False) or (len(self.current_assembly)-len(self.base) != 0 and fixed_base == True) ):
            free_elements = self.order_by_freedom()
            if fixed_base:
                free_elements = [i for i in free_elements if i not in self.base]
            #free_elements = self.get_loosest()
            if len(free_elements) == 0:
                print("No more free elements! Stopping")
                break
            #print(free_elements)

            #loosest = free_elements[0]
            """ Find first loosest element without a successor in the assembly """
            check = False
            for loosest in free_elements:
                if self.check_succession(loosest):
                    check = True
                    break
            if check == False:
                print(free_elements)
                print("No loose elements found without successive element in assembly! Stopping")
                break
            
            elements.append(loosest)
            directions.append(self.free_dict[loosest])
            cum_freedom += len(self.free_dict[loosest])


            self.remove_element(loosest)
            self.combine_all()

        lend = [len(d) for d in directions]
        print(sum(lend))
        print("Min : {}".format(min(lend)))
        print("Max : {}".format(max(lend)))
        print("Ave : {}".format(np.mean(lend)))
        print("Std : {}".format(np.std(lend)))

        return elements, directions

    def reconstruct_from_tree_node(self, node):
        path = node.retrace()
        self.reset_assembly()
        elements = [el.id_e for el in path[1:]]
        directions = []

        for el in elements:
            directions.append(self.free_dict[el])
            self.remove_element(el)
            self.combine_all()

        return elements, directions

        
    """
    Data saving/loading functions
    """

    def save_state(self):
        current_assembly = self.current_assembly
        liaison_matrix = self.liaison_matrix
        current_frees = self.current_frees
        current_blocked = self.current_blocked

        return pickle.dumps([current_assembly, liaison_matrix, current_frees, current_blocked])

    def load_state(self, state):
        current_assembly, liaison_matrix, current_frees, current_blocked = pickle.loads(state)
        self.current_assembly = current_assembly
        self.liaison_matrix = liaison_matrix
        self.current_frees = current_frees
        self.current_blocked = current_blocked
        #self.combine_all()

    def save_assembly(self):
        data = {
            'free' : self.free_directions, 
            'test' : self.test_directions,
            'liaisons' : self.liaisons,
            'centroids' : self.centroids,
            'base' : self.base
        }

        self.save_to_pickle(data)
        print("Saved successfully!")

    def load_tree(self):
        self.tree = self.load_from_pickle(self.tree_pickle)
        return self.tree

    def load_assembly(self):
        data = self.load_from_pickle()
        self.free_directions = data['free']
        self.test_directions = data['test']
        self.liaisons = data['liaisons']
        self.centroids = data['centroids']
        self.base = data['base']

    def save_to_pickle(self, data, file=None):
        if file==None:
            file = self.pickle_file

        with open(file, 'wb') as f:
            pickle.dump(data, f)

    def load_from_pickle(self, file=None):
        if file==None:
            file = self.pickle_file

        with open(file, 'rb') as f:
            data = pickle.load(f)
        
        return data

    """
    Tree display
    """

    def tree_convert_recurse(self, graph, baseNode, baseID=0, labels=['Full']):
        #childID = baseID
        if baseID == 0:
            baseID = graph.add_vertex()
            labels.append('Full')
        for childNode in baseNode.children:
            childID = graph.add_vertex()
            #childID += 1
            labels.append(str(childNode.id_e))
            graph.add_edges([(baseID, childID)])

            labels = self.tree_convert_recurse(graph, childNode, baseID=childID, labels=labels)

        return labels

    def tree_to_igraph(self):
        graph = ig.Graph(directed=True)
        
        nodes = []

        baseNode = self.tree[0].retrace()[0]

        labels = self.tree_convert_recurse(graph, baseNode)
        #graph.vs["label"] = labels
        
        return graph, labels

    def plot_igraph(self, pyplot_=False):
        graph, labels = self.tree_to_igraph()
        print("Prepping to plot")
        nr_vertices = graph.vcount()
        
        if nr_vertices >= 100000:
            print("Too many vertices! This will take aaages! Returning")
            return graph, labels

        labels = labels[1:]
        lay = graph.layout('rt')

        import matplotlib.pyplot as plt
        fig, ax = plt.subplots()
        #plt.invert_yaxis()
        #plt.ylim()
        ig.plot(graph, target=ax, layout = lay, vertex_label=labels)
        ax.invert_yaxis()
        plt.show()

        position = {k: lay[k] for k in range(nr_vertices)}
        for k in position:
            position[k][0] *= 100
            position[k][1] *= 200
        # ig.plot(graph, layout = lay)

        Y = [200*lay[k][1] for k in range(nr_vertices)]
        M = max(Y)

        es = ig.EdgeSeq(graph) # sequence of edges
        E = [e.tuple for e in graph.es] # list of edges

        L = len(position)
        Xn = [position[k][0] for k in range(L)]
        Yn = [2*M-position[k][1] for k in range(L)]
        Xe = []
        Ye = []
        for edge in E:
            Xe+=[position[edge[0]][0],position[edge[1]][0], None]
            Ye+=[2*M-position[edge[0]][1],2*M-position[edge[1]][1], None]

        import plotly.graph_objects as go
        fig = go.Figure()
        fig.add_trace(go.Scatter(x=Xe,
                        y=Ye,
                        mode='lines',
                        line=dict(color='rgb(210,210,210)', width=1),
                        hoverinfo='none'
                        ))
        fig.add_trace(go.Scatter(x=Xn,
                        y=Yn,
                        mode='markers',
                        name='bla',
                        marker=dict(symbol='circle-dot',
                                        size=18,
                                        color='#6175c1',    #'#DB4551',
                                        line=dict(color='rgb(50,50,50)', width=1)
                                        ),
                        text=labels,
                        hoverinfo='text',
                        opacity=0.8
                        ))

        def make_annotations(pos, text, font_size=10, font_color='rgb(250,250,250)'):
            L=len(pos)
            if len(text)!=L:
                raise ValueError('The lists pos and text must have the same len')
            annotations = []
            for k in range(L):
                annotations.append(
                    dict(
                        text=labels[k], # or replace labels with a different list for the text within the circle
                        x=pos[k][0], y=2*M-position[k][1],
                        xref='x1', yref='y1',
                        font=dict(color=font_color, size=font_size),
                        showarrow=False)
                )
            return annotations

        axis = dict(showline=False, # hide axis line, grid, ticklabels and  title
            zeroline=False,
            showgrid=False,
            showticklabels=False,
            )

        fig.update_layout(title= 'Tree with Reingold-Tilford Layout',
                    annotations=make_annotations(position, labels),
                    font_size=12,
                    showlegend=False,
                    xaxis=axis,
                    yaxis=axis,
                    margin=dict(l=40, r=40, b=85, t=100),
                    hovermode='closest',
                    plot_bgcolor='rgb(248,248,248)'
                    )
        if pyplot_:
            fig.show(renderer="browser")

    def plot_liaisons(self):
        graph = ig.Graph()
        graph.add_vertices(self.num_members)
        nr_vertices = graph.vcount()
        labels = list(range(nr_vertices))

        for cnt in range(self.num_members):
            if len(self.liaisons[cnt]) > 0:
                for i in self.liaisons[cnt]:
                    if graph.get_eid(cnt, i, directed=False, error=False) == -1:
                        graph.add_edge(cnt, i)
        graph.vs["label"] = list(range(self.num_members))
        
        layout = graph.layout('circle')
        ig.plot(graph, layout=layout)

        position = {k: layout[k] for k in range(nr_vertices)}

        for k in position:
            position[k][0] *= 200
            position[k][1] *= 200

        # ig.plot(graph, layout = lay)

        Y = [200*layout[k][1] for k in range(nr_vertices)]
        M = max(Y)

        es = ig.EdgeSeq(graph) # sequence of edges
        E = [e.tuple for e in graph.es] # list of edges

        L = len(position)
        Xn = [position[k][0] for k in range(L)]
        Yn = [2*M-position[k][1] for k in range(L)]
        Xe = []
        Ye = []
        for edge in E:
            Xe+=[position[edge[0]][0],position[edge[1]][0], None]
            Ye+=[2*M-position[edge[0]][1],2*M-position[edge[1]][1], None]

        import plotly.graph_objects as go
        fig = go.Figure()
        fig.add_trace(go.Scatter(x=Xe,
                        y=Ye,
                        mode='lines',
                        line=dict(color='rgb(210,210,210)', width=1),
                        hoverinfo='none'
                        ))
        fig.add_trace(go.Scatter(x=Xn,
                        y=Yn,
                        mode='markers',
                        name='bla',
                        marker=dict(symbol='circle-dot',
                                        size=18,
                                        color='#6175c1',    #'#DB4551',
                                        line=dict(color='rgb(50,50,50)', width=1)
                                        ),
                        text=labels,
                        hoverinfo='text',
                        opacity=0.8
                        ))

        def make_annotations(pos, text, font_size=10, font_color='rgb(250,250,250)'):
            L=len(pos)
            if len(text)!=L:
                raise ValueError('The lists pos and text must have the same len')
            annotations = []
            for k in range(L):
                annotations.append(
                    dict(
                        text=labels[k], # or replace labels with a different list for the text within the circle
                        x=pos[k][0], y=2*M-position[k][1],
                        xref='x1', yref='y1',
                        font=dict(color=font_color, size=font_size),
                        showarrow=False)
                )
            return annotations

        axis = dict(showline=False, # hide axis line, grid, ticklabels and  title
            zeroline=False,
            showgrid=False,
            showticklabels=False,
            )

        fig.update_layout(title= 'Liaison node graph',
                    annotations=make_annotations(position, labels),
                    font_size=12,
                    showlegend=False,
                    xaxis=axis,
                    yaxis=axis,
                    margin=dict(l=40, r=40, b=85, t=100),
                    hovermode='closest',
                    plot_bgcolor='rgb(248,248,248)'
                    )
        fig.show(renderer="browser")

    class TreeNode(object):
        def __init__(self, id_e=-1, parent=None, cum_freedom=0, num_left=9999):
            self.id_e = id_e
            self.parent = parent
            self.children = []
            self.num_left = num_left
            self.cum_freedom = cum_freedom
            # if parent != None:
            #     parent.children.append(self)

        def retrace(self):
            sequence = []
            node = self
            while node is not None:
                sequence.append(node)
                node = node.parent
            return sequence[::-1]

        def set_success(self):
            node = self
            while node is not None:
                if node.parent is not None:
                    if node not in node.parent.children:
                        node.parent.children.append(node)
                        node = node.parent
                    else:
                        break
                else:
                    break

        def __str__(self):
            return "Node" + '(' + str(self.id_e) + ', ' + str(len(self.children)) + ')'
        __repr__ = __str__

if __name__ == '__main__':
    def test_removal_and_frees():
        test = Assembly()
        ele = 2
        print("Free {} ".format(len(test.combine_ndfgs(ele))))    
        print("Blocked {} ".format(len(test.free_to_blocking(test.combine_ndfgs(ele)))))
        for neighbour in test.get_full_neighbours(ele):
            print("Removing element {}".format(neighbour))
            test.remove_element(neighbour)
            print("Free {} ".format(len(test.combine_ndfgs(ele))))
            print("Blocked {} ".format(len(test.free_to_blocking(test.combine_ndfgs(ele)))))
        test.reset_assembly()
        print([len(free) for free in test.current_frees])
        print(np.array(test.liaison_matrix))
    
    def get_all_free():
        test = Assembly()
        test.reset_assembly()

        for element in [element for cnt, element in enumerate(test.current_assembly) if len(test.current_frees[cnt]) > 1]:
            print(element)

    def disassemble():
        test = Assembly()
        # print([len(i) for i in test.current_frees])
        # input()
        test.disassembly_tree(60*60*1.5, min_freedom=0)
    #disassemble()

    def check_succession():
        test = Assembly()
        #print(len(test.succession))
        # print(test.precedence[12])
        # print(test.succession[test.precedence[12]])
        # print(test.succession[12])
        for el in [49, 25, 73]:
            print(test.succession[el])

    import cProfile
    cProfile.run('disassemble()')

    def plot_tree():
        test = Assembly()
        test.load_tree()
        test.plot_igraph(pyplot_=True)


    def compare_looseness():
        test = Assembly()

        print("Greedy")
        print("---------")
        _,_a_ = test.disassemble_loosest()


        print("Recursive")
        print("---------")
        tree = test.load_tree()

        #elements, directions = test.disassemble_loosest()

        sorted_tree = sorted(tree, key=lambda x: x.cum_freedom, reverse=True)

        first = sorted_tree[0]
        fst_path = first.retrace()

        lend = [fst_path[1].cum_freedom]

        for cnt in range(1, len(fst_path)):
            lend.append(fst_path[cnt].cum_freedom - fst_path[cnt-1].cum_freedom)
        
        print(sum(lend))
        print("Min : {}".format(min(lend)))
        print("Max : {}".format(max(lend)))
        print("Ave : {}".format(np.mean(lend)))
        print("Std : {}".format(np.std(lend)))
        elements, directions = test.reconstruct_from_tree_node(first)
    
    #compare_looseness()
    #plot_tree()
    # test = Assembly()
    # print(test.precedence)
    # print(test.succession)

    # test = Assembly()
    # test.plot_liaisons()
    # print(test.liaisons)