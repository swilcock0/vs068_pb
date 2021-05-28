import sys
import os
import vs068_pb.config as config

OUT_TO_FILE = True

if OUT_TO_FILE:
    sys.stdout = open(os.path.join(config.src_fldr, "dissassembly_log.txt"), 'w') # Change to 'a' for append

    import atexit
    from datetime import datetime

    load_time = datetime.now()
    dt_string = load_time.strftime("%d/%m/%Y %H:%M:%S")
    print("Logfile time: ", dt_string)

    @atexit.register
    def clean_stdout():
        # Ensure we clean up the logfile when the module is unloaded
        close_time = datetime.now()
        dt_string = close_time.strftime("%d/%m/%Y %H:%M:%S")
        print("Closing logfile at time: ", dt_string)
        sys.stdout.close()    

import pickle
import numpy as np
import time

class TreeNode(object):
    
    def __init__(self, id_e=-1, parent=None, num_left=9999):
        self.id_e = id_e
        self.parent = parent
        self.children = []
        self.num_left = num_left

    def retrace(self):
        sequence = []
        node = self
        while node is not None:
            sequence.append(node)
            node = node.parent
        return sequence[::-1]

    def __str__(self):
        return "Node" + '(' + str(self.id_e) + ', ' + str(len(self.children)) + ')'
    __repr__ = __str__


class Assembly(object):
    def __init__(self, free_directions=None, test_directions=None, liaisons=None, centroids=None):
        self.pickle_file = os.path.join(config.src_fldr, "pickles", "assembly.pickle")

        if free_directions == None or test_directions == None or liaisons == None or centroids == None:
            print("Loading directions...")
            self.load_assembly()
        else:
            print("Initialising with new directions...")

            # Convert to tuples for easing set operations
            #print(free_directions)
            self.free_directions = [[[tuple(direction) for direction in face] for face in element] for element in free_directions]
            self.test_directions = [tuple(direction) for direction in test_directions]
            self.liaisons = liaisons
            self.centroids = centroids

            self.save_assembly()


        
        self.base = set([0, 23, 24, 47, 48, 71, 72, 95])
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
        Build lists of precedence and succession relationships (basic, based on picking the lowest neighbour) 
        """
        # lst = []
        # lst.
        self.precedence = []
        for el in range(self.num_members):
            neighbours = self.get_full_neighbours(el)
            ngbr_heights = [self.centroids[p][2] for p in neighbours]
            # ngbr_cntrs = [self.centroids[p] for p in neighbours]
            
            # neighbour_dict = {}
            # for cnt, n in enumerate(neighbours):
            #     neighbour_dict.update(
            #         {
            #             n : {
            #                 'x' : ngbr_cntrs[0],
            #                 'y' : ngbr_cntrs[1],
            #                 'z' : ngbr_cntrs[2]
            #             }
            #         }
            #     )
            # sorted_by_height = sorted(neighbours, ngbr_cntrs), key=lambda x:x[2])
            self.precedence.append(neighbours[ngbr_heights.index(min(ngbr_heights))])

        self.succession = []

        for el in range(self.num_members):
            if el in self.precedence:
                self.succession.append(self.precedence.index(el))
            else:
                self.succession.append(-1)
                

    def combine_all(self):
        self.current_frees = [self.combine_ndfgs(i) for i in self.current_assembly]
        self.free_dict = {i : self.combine_ndfgs(i) for i in self.current_assembly}
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

    def display(self, nodes):
        if nodes is None:
            return None
        return list(map(lambda n: n.id_e, nodes))


    def disassemble(self, state=None, nodes=None, start_time=-1, end_time=-1, depth=0):
        # Build first nodes
        if start_time == -1 or end_time == -1:
            raise Exception("Require a start and end time (or it will go on forever!)")
        if time.time()-start_time > end_time:
            return
        if nodes == None:
            nodes = [TreeNode()]
            base = True
            print("Base")
        else:
            base = False
        
        if state == None:
            state = self.save_state()

        success = False

        free_elements = []
        free_dir_lens = []
        assert len(self.current_frees)==len(self.current_assembly)
        for i in range(len(self.current_assembly)):
            if len(self.current_frees[i]) > 1:
                free_elements.append(self.current_assembly[i])
                free_dir_lens.append(len(self.current_frees[i])) 
        #print(free_elements)
        zipped_lists = zip(free_dir_lens, free_elements)
        sorted_zipped_lists = sorted(zipped_lists)

        free_elements = [element for _, element in sorted_zipped_lists]
        print(free_elements)
        print(depth)
        depth += 1
        if free_elements != []:
            for element in free_elements:    
                if success:
                    return

                if time.time() - start_time > end_time:
                    success = True

                self.load_state(state)
                self.remove_element(element)
                self.combine_all()
                state_new = self.save_state()
                parent = nodes[-1]
                child = TreeNode(id_e=element, parent=parent, num_left=len(self.current_assembly))
                nodes.append(child)

                if len(self.current_assembly) == 0:
                    print("Found full disassembly")
                    success = True                    
                else:
                    for n, s in self.disassemble(state=state_new, nodes=nodes, start_time=start_time, end_time=end_time, depth=depth):
                        for n_i in n:
                            nodes.append(n_i)
                        if s:
                            success = True

                yield nodes, success
        else:
            return

    def order_by_freedom(self):
        free_elements = []
        free_dir_lens = []
        
        assert len(self.current_frees)==len(self.current_assembly)
        for i in range(len(self.current_assembly)):
            if len(self.current_frees[i]) > 1:
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
        if self.succession[element] == -1 or self.succession[element] in self.base:
            return True
        if self.succession[element] in self.current_assembly:
            return False
            if output:
                print(self.succession[element])
        else:
            return True


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


            self.remove_element(loosest)
            self.combine_all()
        #print(elements)
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

    def save_assembly(self):
        data = {
            'free' : self.free_directions, 
            'test' : self.test_directions,
            'liaisons' : self.liaisons,
            'centroids' : self.centroids
        }

        self.save_to_pickle(data)
        print("Saved successfully!")

    def load_assembly(self):
        data = self.load_from_pickle()
        self.free_directions = data['free']
        self.test_directions = data['test']
        self.liaisons = data['liaisons']
        self.centroids = data['centroids']

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



# if __name__ == '__main__':
#     def test_removal_and_frees():
#         test = Assembly()
#         ele = 2
#         print("Free {} ".format(len(test.combine_ndfgs(ele))))    
#         print("Blocked {} ".format(len(test.free_to_blocking(test.combine_ndfgs(ele)))))
#         for neighbour in test.get_full_neighbours(ele):
#             print("Removing element {}".format(neighbour))
#             test.remove_element(neighbour)
#             print("Free {} ".format(len(test.combine_ndfgs(ele))))
#             print("Blocked {} ".format(len(test.free_to_blocking(test.combine_ndfgs(ele)))))
#         test.reset_assembly()
#         print([len(free) for free in test.current_frees])
#         print(np.array(test.liaison_matrix))
    
#     def get_all_free():
#         test = Assembly()
#         test.reset_assembly()

#         for element in [element for cnt, element in enumerate(test.current_assembly) if len(test.current_frees[cnt]) > 1]:
#             print(element)

#     def disassemble():
#         test = Assembly()
        
#         elements, _ = test.disassemble_loosest()
#         print(elements)
#         print(len(elements))

#     def check_succession():
#         test = Assembly()
#         #print(len(test.succession))
#         # print(test.precedence[12])
#         # print(test.succession[test.precedence[12]])
#         # print(test.succession[12])
#         for el in [49, 25, 73]:
#             print(test.succession[el])
#     disassemble()