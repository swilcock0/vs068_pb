import vs068_pb.config as config
from vs068_pb.utils import set_joint_states
import pybullet as p
import random

class Scene(object):
    def __init__(self, physicsClientId=-1, botId=-1):
        self.collision_objects = {}
        self.object_counter = 1
        self.object_counter_max = 1
        self.allowed = []
        self.physicsClientId = physicsClientId
        self.botId = botId

    def change_physics_clients(self, physicsClientId):
        self.physicsClientId = physicsClientId

        for g in self.collision_objects:
            self.collision_objects[g].change_physics_client(physicsClientId)

    def assign_bot(self, botId):
        self.botId = botId

    def add_object(self, geometry):
        self.collision_objects.update({self.object_counter_max : geometry})
        self.object_counter += 1
        self.object_counter_max += 1
        geometry.add_collision()
        geometry.add_visual()
        

    def remove_object(self, object_count):
        self.collision_objects[object_count].clear()
        del self.collision_objects[object_count]
        self.object_counter -= 1

    def clear_all(self):
        for g in self.collision_objects:
            g.clear()

        self.collision_objects = {}
        self.object_counter = 1
        self.object_counter_max = 1

    def initialise_allowed(self):
        self.allowed = []
        if self.botId != -1:
            set_joint_states(self.physicsClientId, self.botId, config.info.free_joints, [0]*6, [0]*6)
        p.stepSimulation(self.physicsClientId)

        for contact in (p.getContactPoints(physicsClientId=self.physicsClientId)):
            if (([contact[3], contact[4]] not in config.NEVER_COLLIDE_NUMS) and contact[1] != contact[2]):                
                self.allowed.append(contact[:5])

    def check_contacts_against_init(self, contact, allowed=[]):
        if allowed == []:
            allowed = self.allowed

        if contact[:5] in allowed:
            # if config.TEST_COLLISIONS_VERBOSE:
            #     print("Allowed:")
            #     pretty_print_contact(contact)
            return False
        else:
            return True


    def check_collisions(self, q):
        joints = config.info.free_joints+config.FINGER_JOINTS
        if self.botId != -1:
            set_joint_states(self.physicsClientId, self.botId, joints, list(q)+[0]*2, [0]*8)
        p.stepSimulation(self.physicsClientId)
        allowedContacts = True

        contacts = p.getContactPoints(physicsClientId=self.physicsClientId)

        for contact in (p.getContactPoints(physicsClientId=self.physicsClientId)):
            if (([contact[3], contact[4]] not in config.NEVER_COLLIDE_NUMS) and contact[1] != contact[2]) and self.check_contacts_against_init(contact[:5]):                
                allowedContacts = False
                # if config.TEST_COLLISIONS_VERBOSE:
                #     print("Blocked")
                #     pretty_print_contact(contact)

        return not(allowedContacts)

class Geometry(object):
    def __init__(self, safety_margin=0.05, colour=-1, physicsClientId=0):
        self.pose = [[0,0,0], [0,0,0,1]]
        self.geometry_type = -1
        self.id_collision = -1
        self.id_visual = -1
        self.physicsClientId = physicsClientId
        self.safety_margin = safety_margin
        self.mass = 1
        if colour == -1:
            self.rgbaColor = [random.random(), random.random(), random.random(), 1.0]
        else:
            self.rgbaColor = colour

        self.type_dict = {
            p.GEOM_SPHERE : "Sphere",
            p.GEOM_BOX : "Box",
            p.GEOM_CAPSULE : "Capsule",
            p.GEOM_CYLINDER : "Cylinder",
            p.GEOM_PLANE : "Plane",
            p.GEOM_MESH : "Mesh",
            p.GEOM_HEIGHTFIELD : "Heightfield"
        }
        self.repr_additional_info = None


    def __repr__(self):
        if self.geometry_type == -1:
            return "Geometry (currently undefined)"
        else:
            return "Geometry(Type: " + self.type_dict[self.geometry_type] + ", Pose: " + str(self.pose) + ")"


    def change_physics_client(self, physicsClientId):
        self.physicsClientId = physicsClientId
        self.add_collision()
        self.add_visual()

    def clear(self):
        p.removeBody(self.id_collision, physicsClientId=self.physicsClientId)
        p.removeBody(self.id_visual, physicsClientId=self.physicsClientId)                

    def hide_collision_visual(self):
        colour = self.rgbaColor
        colour[3] = 0.0
        p.changeVisualShape(self.id_collision, -1, rgbaColor=colour, physicsClientId=self.physicsClientId)

    def hide_visual(self):
        colour = self.rgbaColor
        colour[3] = 0.0
        p.changeVisualShape(self.id_visual, -1, rgbaColor=colour, physicsClientId=self.physicsClientId)

    def show_collision_visual(self):
        colour = self.rgbaColor
        colour[3] = 0.3
        p.changeVisualShape(self.id_collision, -1, rgbaColor=colour, physicsClientId=self.physicsClientId)

    def show_visual(self):
        colour = self.rgbaColor
        colour[3] = 1.0
        p.changeVisualShape(self.id_visual, -1, rgbaColor=colour)

    def define_mesh(self, filename, pose_centre=[[0,0,0], [0,0,0,1]], scale=1):
        self.geometry_type = p.GEOM_MESH
        self.pose = pose_centre

        pos = pose_centre[0]
        orn = pose_centre[1]
        file_id = filename
        meshScale = [scale,scale,scale]
        if self.safety_margin < 1:
            print("Safety margin for meshes is multiplicative! Changing to {:.2%} increase".format(self.safety_margin))
            self.safety_margin += 1
        
        safetyScale = [scale*self.safety_margin, scale*self.safety_margin, scale*self.safety_margin]

        def add_collision():
            colour = self.rgbaColor
            colour[3] = 0.3

            mesh_col = p.createCollisionShape(p.GEOM_MESH, 
                            physicsClientId=self.physicsClientId, 
                            fileName = file_id, 
                            meshScale=safetyScale
                            )

            mesh_coll_vis = p.createVisualShape(p.GEOM_MESH, 
                            physicsClientId=self.physicsClientId, 
                            fileName = file_id, 
                            meshScale=safetyScale,
                            rgbaColor = colour
                            )

            self.id_collision = p.createMultiBody(baseMass=self.mass, baseCollisionShapeIndex=mesh_col, baseVisualShapeIndex=mesh_coll_vis, basePosition = pos, baseOrientation=orn, physicsClientId=self.physicsClientId)
            return self.id_collision

        def add_visual():
            colour = self.rgbaColor
            colour[3] = 1.0

            mesh_vis = p.createVisualShape(p.GEOM_MESH, 
                            physicsClientId=self.physicsClientId, 
                            fileName = file_id, 
                            meshScale=meshScale,
                            rgbaColor = colour
                            )

            self.id_visual = p.createMultiBody(baseMass=self.mass, baseVisualShapeIndex=mesh_vis, basePosition = pos, baseOrientation=orn, physicsClientId=self.physicsClientId)
            return self.id_visual

        self.add_collision = add_collision
        self.add_visual = add_visual

    def define_sphere(self, pose_centre=[[0,0,0], [0,0,0,1]], radius=1):
        self.geometry_type = p.GEOM_SPHERE
        self.pose = pose_centre

        pos = pose_centre[0]
        orn = pose_centre[1]
        rad = radius

        def add_collision():
            colour = self.rgbaColor
            colour[3] = 0.3

            radius = rad + self.safety_margin
            sphere_col = p.createCollisionShape(p.GEOM_SPHERE, 
                            physicsClientId=self.physicsClientId, 
                            radius = radius, 
                            )

            sphere_coll_vis = p.createVisualShape(p.GEOM_SPHERE, 
                            physicsClientId=self.physicsClientId, 
                            radius = radius, 
                            rgbaColor=colour
                            )

            self.id_collision = p.createMultiBody(baseMass=self.mass, baseCollisionShapeIndex=sphere_col, baseVisualShapeIndex=sphere_coll_vis, basePosition = pos, baseOrientation=orn, physicsClientId=self.physicsClientId)
            return self.id_collision

        def add_visual():
            colour = self.rgbaColor
            colour[3] = 1.0
            radius = rad
            
            sphere_vis = p.createVisualShape(p.GEOM_SPHERE, 
                            physicsClientId=self.physicsClientId, 
                            radius = radius, 
                            rgbaColor=colour
                            )

            self.id_visual = p.createMultiBody(baseMass=self.mass, baseVisualShapeIndex=sphere_vis, basePosition = pos, baseOrientation=orn, physicsClientId=self.physicsClientId)
            return self.id_visual

        self.add_collision = add_collision
        self.add_visual = add_visual

    
    def define_box(self, pose_centre=[[0,0,0], [0,0,0,1]], dimensions=[1,1,1]):
        self.geometry_type = p.GEOM_BOX
        self.pose = pose_centre

        pos = pose_centre[0]
        orn = pose_centre[1]

        def add_collision():
            colour = self.rgbaColor
            colour[3] = 0.3

            box_lower = [pos[c] - dimensions[c]/2-self.safety_margin for c in range(3)]
            box_upper = [pos[c] + dimensions[c]/2+self.safety_margin for c in range(3)]
            box_halfs = [abs(box_upper[c] - box_lower[c])/2 for c in range(3)]
            box_centre = [box_lower[c] + box_halfs[c] for c in range(3)] 

            box_col = p.createCollisionShape(p.GEOM_BOX, 
                            physicsClientId=self.physicsClientId, 
                            halfExtents = box_halfs, 
                            )

            box_coll_vis = p.createVisualShape(p.GEOM_BOX, 
                            physicsClientId=self.physicsClientId, 
                            halfExtents = box_halfs, 
                            rgbaColor=colour
                            )

            self.id_collision = p.createMultiBody(baseMass=self.mass, baseCollisionShapeIndex=box_col, baseVisualShapeIndex=box_coll_vis, basePosition = box_centre, baseOrientation=orn, physicsClientId=self.physicsClientId)
            return self.id_collision

        def add_visual():
            colour = self.rgbaColor
            colour[3] = 1.0
            box_lower = [pos[c] - dimensions[c]/2 for c in range(3)]
            box_upper = [pos[c] + dimensions[c]/2 for c in range(3)]
            box_halfs = [abs(box_upper[c] - box_lower[c])/2 for c in range(3)]
            box_centre = [box_lower[c] + box_halfs[c] for c in range(3)] 

            box_vis = p.createVisualShape(p.GEOM_BOX, 
                            physicsClientId=self.physicsClientId, 
                            halfExtents = box_halfs, 
                            rgbaColor=colour
                            )

            self.id_visual = p.createMultiBody(baseMass=self.mass, baseVisualShapeIndex=box_vis, basePosition = box_centre, baseOrientation=orn, physicsClientId=self.physicsClientId)
            return self.id_visual

        self.add_collision = add_collision
        self.add_visual = add_visual
    

    def define_plane(self, pose_centre=[[0,0,0], [0,0,0,1]]):
        self.geometry_type = p.GEOM_PLANE
        self.pose = pose_centre

        pos = pose_centre[0]
        orn = pose_centre[1]

        def add_collision():
            colour = self.rgbaColor
            colour[3] = 0.3
            dimensions = [999,999,0.0001]
            box_lower = [pos[c] - dimensions[c]/2-self.safety_margin for c in range(3)]
            box_upper = [pos[c] + dimensions[c]/2+self.safety_margin for c in range(3)]
            box_halfs = [abs(box_upper[c] - box_lower[c])/2 for c in range(3)]
            box_centre = [box_lower[c] + box_halfs[c] for c in range(3)] 

            plane_box_col = p.createCollisionShape(p.GEOM_BOX, 
                            physicsClientId=self.physicsClientId, 
                            halfExtents = box_halfs, 
                            )

            plane_box_vis = p.createVisualShape(p.GEOM_BOX, 
                            physicsClientId=self.physicsClientId, 
                            halfExtents = box_halfs, 
                            rgbaColor=colour
                            )

            self.id_collision = p.createMultiBody(baseMass=self.mass, baseCollisionShapeIndex=plane_box_col, baseVisualShapeIndex=plane_box_vis, basePosition = box_centre, baseOrientation=orn, physicsClientId=self.physicsClientId)
            return self.id_collision

        def add_visual():
            dimensions = [999,999,0.0001]
            box_lower = [pos[c] - dimensions[c]/2 for c in range(3)]
            box_upper = [pos[c] + dimensions[c]/2 for c in range(3)]
            box_halfs = [abs(box_upper[c] - box_lower[c])/2 for c in range(3)]
            box_centre = [box_lower[c] + box_halfs[c] for c in range(3)] 
            
            colour = self.rgbaColor
            colour[3] = 0.3

            plane_box_vis = p.createVisualShape(p.GEOM_BOX, 
                            physicsClientId=self.physicsClientId, 
                            halfExtents = box_halfs, 
                            rgbaColor=colour
                            )

            self.id_visual = p.createMultiBody(baseMass=self.mass, baseVisualShapeIndex=plane_box_vis, basePosition = box_centre, baseOrientation=orn, physicsClientId=self.physicsClientId)
            return self.id_visual

        self.add_collision = add_collision
        self.add_visual = add_visual



'''

class Scene(object):
    def __init__(self):



    def pretty_print_contact(contact, boxes_only=False):
        if boxes_only:
            reversed_dictionary = {value : key for (key, value) in bodies.items()}

            if (contact[1] == reversed_dictionary["Floor"] or contact[1] == reversed_dictionary["Bot"]) and (contact[2] == reversed_dictionary["Floor"] or contact[2] == reversed_dictionary["Bot"]):
                return False

        contact_a_name = bodies.get(contact[1], "Unknown")
        contact_b_name = bodies.get(contact[2], "Unknown")
        link_a_id = contact[3]
        link_b_id = contact[4]
        print("Contact: {}:{} - {}:{}".format(contact_a_name, link_a_id, contact_b_name, link_b_id))
        return True

    if allowed != []:
        print("Initial collisions : ")
        for contact in allowed:
            pretty_print_contact(contact)
        print("... will be ignored throughout.")

    def check_contacts_against_init(contact, allowed=allowed):
        if contact[:5] in allowed:
            if config.TEST_COLLISIONS_VERBOSE:
                print("Allowed:")
                pretty_print_contact(contact)
            return False
        else:
            return True


    def check_collisions(q):
        joints = config.info.free_joints+config.FINGER_JOINTS
        set_joint_states(cid, botId, joints, list(q)+[0]*2, [0]*8)
        p.stepSimulation(cid)
        allowedContacts = True

        contacts = p.getContactPoints(physicsClientId=cid)

        for contact in (p.getContactPoints(physicsClientId=cid)):
            if (([contact[3], contact[4]] not in config.NEVER_COLLIDE_NUMS) and contact[1] != contact[2]) and check_contacts_against_init(contact[:5]):                
                allowedContacts = False
                if config.TEST_COLLISIONS_VERBOSE:
                    print("Blocked")
                    pretty_print_contact(contact)

        return not(allowedContacts)






    def create_box_collisions(dims, pos, safety=0.05):
   
    # Creates box collision and visual functions

    # Example usage:
    # dims = [[0.2, 0.2, 0.2], [0.1, 0.1, 1.2], [0.1, 0.1, 1.2]]
    # pos = [[0.3,0.3,0.7], [-0.25, -0.25, 0.6], [-0.25, 0.25, 0.6]]
    # collision_fn, visual_fn = create_box_collisions(dims, pos, safety=0.05)
    # Gives a 5cm safety boundary on sides

    # Remember to Disconnect after using or their may be artifacts in display
    

    Disconnect()
    botId, cid = quick_load_bot(mode=p.DIRECT)
    planeId = loadFloor(cid)

    bodies = {botId : "Bot", planeId : "Floor"}

    allowed = []

    for i in range(len(dims)):
        box_lower = [pos[i][j] - dims[i][j]/2-safety for j in range(3)]
        box_upper = [pos[i][j] + dims[i][j]/2+safety for j in range(3)]
        box_halfs = [abs(box_upper[i] - box_lower[i])/2 for i in range(3)]
        box_centre = [box_lower[i] + box_halfs[i] for i in range(3)] 

        #print(box_lower, box_centre, box_upper, box_halfs)

        box_col = p.createCollisionShape(p.GEOM_BOX, 
                            physicsClientId=cid, 
                            halfExtents = box_halfs, 
                            )

        test_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=box_col, basePosition = box_centre, physicsClientId=cid)
        bodies.update({test_body : "Box_"+str(i+1)})
    
    set_joint_states(cid, botId, config.info.free_joints, [0]*6, [0]*6)
    p.stepSimulation(cid)

    for contact in (p.getContactPoints(physicsClientId=cid)):
        if (([contact[3], contact[4]] not in config.NEVER_COLLIDE_NUMS) and contact[1] != contact[2]):                
            allowed.append(contact[:5])
    
    print("Scene : {}".format(bodies))
    
    



    def create_visual_fn(cid, collision_boxes=False):
        vis_col = []

        for i in range(len(dims)):
            box_lower = [pos[i][j] - dims[i][j]/2 for j in range(3)]
            box_upper = [pos[i][j] + dims[i][j]/2 for j in range(3)]
            box_halfs = [abs(box_upper[i] - box_lower[i])/2 for i in range(3)]
            box_centre = [box_lower[i] + box_halfs[i] for i in range(3)] 

            colour = [0.0, 0.0, 0.0, 1.0]
            colour[i%3] = 0.8

            box_vis = p.createVisualShape(p.GEOM_BOX, 
                                rgbaColor=colour, 
                                physicsClientId=cid, 
                                halfExtents = box_halfs, 
                                )

            box_col = p.createCollisionShape(p.GEOM_BOX, 
                                physicsClientId=cid, 
                                halfExtents = box_halfs, 
                                )

            test_body = p.createMultiBody(baseMass=1, baseVisualShapeIndex=box_vis, baseCollisionShapeIndex=box_col, basePosition = box_centre, physicsClientId=cid)

            if collision_boxes:
                box_lower = [pos[i][j] - dims[i][j]/2-safety for j in range(3)]
                box_upper = [pos[i][j] + dims[i][j]/2+safety for j in range(3)]
                box_halfs = [abs(box_upper[i] - box_lower[i])/2 for i in range(3)]
                box_centre = [box_lower[i] + box_halfs[i] for i in range(3)] 
                colour[3] = 0.2

                box_vis = p.createVisualShape(p.GEOM_BOX, 
                                rgbaColor=colour, 
                                physicsClientId=cid, 
                                halfExtents = box_halfs, 
                                )

                test_body = p.createMultiBody(baseMass=1, baseVisualShapeIndex=box_vis, basePosition = box_centre, physicsClientId=cid)
                vis_col.append(test_body)
        return vis_col

    return collision_fn, create_visual_fn
'''